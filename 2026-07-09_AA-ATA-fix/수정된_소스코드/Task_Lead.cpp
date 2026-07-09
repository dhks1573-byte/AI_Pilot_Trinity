#include "Task_Lead.h"
#include <iostream>

PortsList Action::Task_Lead::providedPorts()
{
	return {
			InputPort<CPPBlackBoard*>("BB")
	};
}

NodeStatus Action::Task_Lead::tick()
{
	Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

	Vector3 MyLocation = (*BB)->MyLocation_Cartesian;
	Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;
	Vector3 TargetForward = (*BB)->TargetForwardVector;

	TargetForward.normalize();
	Vector3 TargetVelocity = TargetForward * (double)(*BB)->TargetSpeed_MS;

	double Distance = MyLocation.distance(TargetLocation);
	double MySpeed = (*BB)->MySpeed_MS;
	if (MySpeed < 1.0)
	{
		MySpeed = 1.0;
	}

	double InterceptTime = Distance / MySpeed;
	// 너무 먼 거리/저속 상황에서 리드포인트가 과도하게 튀는 것을 방지
	if (InterceptTime > 8.0)
	{
		InterceptTime = 8.0;
	}

	// 강하각 클램프 pursuit: VP.Z를 자기 고도로 고정(flat)하면 컨트롤러가 절대 못 내려오므로
	// 타겟 고도를 따라가되 급강하/급상승은 각도로 제한한다.
	// 상승(회복 여유 충분)은 관대하게(~26.5도), 강하는 보수적으로(~11도) 비대칭 클램프.
	// 롤 비대칭 버그 수정 후 강하가 너무 효율적이 되어 회복 여유 없이 MinAlt를 뚫고 추락하는
	// 사고가 발생해서, 강하 쪽만 좁히고 바닥도 ClimbOut 트리거(3000m)보다 위로 올려둔다.
	Vector3 LeadPoint = TargetLocation + TargetVelocity * InterceptTime;
	double climbSlope = Distance * 0.5;
	double diveSlope = Distance * 0.2;
	double minZ = MyLocation.Z - diveSlope;
	double maxZ = MyLocation.Z + climbSlope;
	if (LeadPoint.Z < minZ) LeadPoint.Z = minZ;
	if (LeadPoint.Z > maxZ) LeadPoint.Z = maxZ;
	if (LeadPoint.Z < 3500.0) LeadPoint.Z = 3500.0;

	(*BB)->VP_Cartesian = LeadPoint;

	// 2026-07-09: Task_Pure에서 검증된 속도차 기반 스로틀 제어(속도차>20m/s ->
	// throttle 0.7)를 이 원거리 접근 태스크에도 적용해봤으나, 20시드 배치에서
	// mean -99.06 -> -115.05로 뚜렷한 악화(패배 재발은 없었음) 확인되어 원복함.
	// 결론: 폐쇄율 관리는 WEZ 근접 단계(Task_Pure)에서만 유효하고, 원거리
	// 접근 단계에서 미리 감속하는 건 오히려 접근 자체의 에너지/시간을 낭비해서
	// 손해 -- Task_Pure만 스로틀을 건드리고 Task_Lead는 항상 최대 유지가 맞음.
	// [[session-2026-07-09-aip-dogfight-ata-vs-aa]] 후속5/6 참고.

	static int __dbg[2] = { 0, 0 };
	int __t = ((*BB)->Team == BLUE) ? 0 : 1;
	if (++__dbg[__t] % 30 == 0) std::cerr << "[ACTIVE] [" << ((*BB)->Team == BLUE ? "BLUE" : "RED") << "] Lead Z=" << MyLocation.Z << " VPZ=" << LeadPoint.Z << std::endl;

	return NodeStatus::SUCCESS;
}
