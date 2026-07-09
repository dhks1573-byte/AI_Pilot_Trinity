// 상대 에너지 + 기하학적 조준각(ATA)을 근거로 BB->BFM(OBFM/DBFM/HABFM)을 갱신하는 서비스 노드
// AIAA 2023 "Manual-Based Automated Maneuvering Decisions for Air-to-Air Combat" 구조 참고.
// 이 프로젝트 템플릿에 이미 있던 BFM_Mode/DECO_BFMCheck 인프라를 채우는 목적, 2026-07-06

#include "BFMClassifier.h"
#include <iostream>

namespace Action
{
	PortsList BFMClassifier::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB")
		};
	}

	NodeStatus BFMClassifier::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		Vector3 MyLocation     = (*BB)->MyLocation_Cartesian;
		Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;
		Vector3 MyForward      = (*BB)->MyForwardVector;

		// MyATA: 내 기수와 타겟 방향(LOS) 사이의 각. DECO_TailThreatCheck와 동일 공식.
		// 값이 크면(타겟이 내 뒤쪽) 위협, 값이 작으면(타겟이 내 앞쪽) 조준 기회.
		// (참고: 첫 시도에서는 "타겟이 나를 조준 중인지"를 별도 계산해 DBFM 판정에 썼으나,
		//  상대가 매틱 순수추적만 하는 봇이라 항상 낮게 나와 DBFM으로 편향되는 버그가 있었음.
		//  기존에 검증된 TailThreatCheck/AspectAngleCheck와 동일한 두 primitive로 교체함.)
		Vector3 MyToTarget = TargetLocation - MyLocation;
		float MyATA = MyToTarget.angleBetween(MyForward) * 57.2958f;

		// AA: 내가 타겟의 꼬리를 얼마나 잘 물었는지. AspectAngleUpdate 서비스가 이미 갱신해둠.
		float AA = (*BB)->MyAspectAngle_Degree;

		// 상대 특성에너지 차이 (EM 차트 비교의 단순화 근사치), 애매한 중간 지대에서만 사용
		const double G = 9.81;
		double MySpecificEnergy     = MyLocation.Z + ((*BB)->MySpeed_MS * (*BB)->MySpeed_MS) / (2.0 * G);
		double TargetSpecificEnergy = TargetLocation.Z + ((*BB)->TargetSpeed_MS * (*BB)->TargetSpeed_MS) / (2.0 * G);
		double dE = MySpecificEnergy - TargetSpecificEnergy;

		// TAILTHREAT_THRESHOLD: 기존 DECO_TailThreatCheck와 동일 임계값(150).
		// (처음엔 120으로 더 일찍 감지하도록 했으나, baseline보다 훨씬 잦은 DBFM
		// 오발동으로 불필요한 회피 기동을 유발해 궤적이 크게 갈라지는 원인으로 확인됨 -> 원복)
		// AA_THRESHOLD: 기존 AspectCheck_NotBehind와 동일 임계값
		// ENERGY_THRESHOLD: meter 단위 특성에너지 차이
		// TACTICAL_RANGE: 이 거리 밖에서는 순간적인 각도 관계가 실제 위협을 의미하지 않음
		// (Rule_Trinity의 TailThreat_RangeGate가 항상 3000m 게이트를 같이 쓰는 것과 동일한 이유)
		const float TAILTHREAT_THRESHOLD = 150.0f;
		const float AA_THRESHOLD = 60.0f;
		const double ENERGY_THRESHOLD = 300.0;
		const float TACTICAL_RANGE = 3000.0f;

		float Distance = (*BB)->Distance;
		bool InTacticalRange = (Distance < TACTICAL_RANGE);

		BFM_Mode NewBFM;

		// 2026-07-09: AA<60(내가 이미 타겟 꼬리를 물고 있음) 체크를 최우선으로 승격.
		// 기존엔 TAILTHREAT(MyATA>150)/ENERGY(dE<-300) 체크가 먼저였는데, 이 둘은
		// "내 기수가 타겟을 향하는가(MyATA)"와 "순간 에너지"만 보고 "이미 좋은 위치
		// (AA<60)"인지는 전혀 고려하지 않아서, 추격 중 자연스러운 오버슈트(위치는
		// 완벽한데 기수만 잠깐 딴 곳을 보는 상황, MyATA>150)나 선회로 인한 일시적
		// 에너지 열세만으로 공격을 중단하고 DBFM(Task_Extend, 이탈+상승)으로 빠지는
		// 오발동이 실측 확인됨(한 에피소드에서 DBFM 171건 중 110건이 AA<60인데도
		// 발동, [[session-2026-07-09-aip-dogfight-ata-vs-aa]]). 이미 위치 우위(AA<60)를
		// 잡았다면 절대 방어로 후퇴하지 않고 계속 공격 유지 -> Task_Pure/Task_Lead가
		// 이번 세션에 신설된 boresight 각도제한 우회(CPPBehaviorTree.cpp)의 도움을
		// 받아 기수를 마저 돌리도록 기회를 준다.
		if (AA < AA_THRESHOLD)
		{
			// 내가 타겟의 꼬리를 물고 있음 -> 공격(최우선, 절대 안 뺏김)
			NewBFM = OBFM;
		}
		else if (InTacticalRange && MyATA > TAILTHREAT_THRESHOLD)
		{
			// 타겟이 내 뒤쪽에 있고, 실제로 위협이 될 만큼 가까움 -> 방어
			NewBFM = DBFM;
		}
		else if (InTacticalRange && dE < -ENERGY_THRESHOLD)
		{
			NewBFM = DBFM;
		}
		else if (dE > ENERGY_THRESHOLD)
		{
			NewBFM = OBFM;
		}
		else
		{
			// 원거리이거나, 양쪽 다 애매한 빔/머지 상황 -> 중립(접근/각도잡기)
			NewBFM = HABFM;
		}

		(*BB)->BFM = NewBFM;

		static int __dbg[2] = { 0, 0 };
		int __t = ((*BB)->Team == BLUE) ? 0 : 1;
		if (++__dbg[__t] % 30 == 0)
		{
			const char* modeStr = (NewBFM == OBFM) ? "OBFM" : (NewBFM == DBFM) ? "DBFM" : (NewBFM == HABFM) ? "HABFM" : "OTHER";
			std::cerr << "[BFM] team=" << (*BB)->Team << " mode=" << modeStr
				<< " MyATA=" << MyATA << " AA=" << AA << " dE=" << dE
				<< " Dist=" << (*BB)->Distance << std::endl;
		}

		return NodeStatus::SUCCESS;
	}

}
