#include "Task_Evade.h"
#include <iostream>
#include <cmath>

PortsList Action::Task_Evade::providedPorts()
{
	return {
			InputPort<CPPBlackBoard*>("BB")
	};
}

NodeStatus Action::Task_Evade::tick()
{
	Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

	Vector3 MyLocation = (*BB)->MyLocation_Cartesian;
	Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;
	Vector3 MyRight = (*BB)->MyRightVector;
	Vector3 MyUp = (*BB)->MyUpVector;

	Vector3 LOSToTarget = TargetLocation - MyLocation;
	LOSToTarget.normalize();

	// Use world-up to avoid diving when aircraft is banked/inverted
	Vector3 WorldUp(0.0, 0.0, 1.0);

	double Side = MyRight.dot(LOSToTarget);

	// 2026-07-12 수정: 타겟이 정확히 내 6시(Evade 발동조건 자체)에 있을 때는
	// Side가 구조적으로 0 근처라 매틱 부호가 뒤집혀 브레이크 방향을 못 정하는
	// chattering이 실측 확인됨([[session-2026-07-12-evasion-root-cause]], 58초
	// 스트릭에서 turnSign 21% 반전, 반전률과 스트릭 길이가 뚜렷이 상관).
	// |Side|가 임계치 미만이면 새로 판단하지 않고 직전 방향을 유지(hysteresis)해서
	// 한쪽으로 계속 밀어붙이는 각속도가 쌓이도록 한다. 확실히 한쪽으로 넘어갔을
	// 때만(|Side|>=0.15) 방향을 갱신.
	const double SIDE_HYSTERESIS = 0.15;
	static double __lastTurnSign[2] = { 1.0, 1.0 };
	int __t = ((*BB)->Team == BLUE) ? 0 : 1;
	double TurnSign;
	if (std::abs(Side) < SIDE_HYSTERESIS)
	{
		TurnSign = __lastTurnSign[__t];
	}
	else
	{
		TurnSign = (Side >= 0.0) ? 1.0 : -1.0;
	}
	__lastTurnSign[__t] = TurnSign;

	Vector3 BreakDirection = MyRight * TurnSign;
	BreakDirection.normalize();

	(*BB)->VP_Cartesian = MyLocation + BreakDirection * 3000.0;

	static int __dbg[2] = { 0, 0 };
	if (++__dbg[__t] % 30 == 0) std::cerr << "[ACTIVE] [" << ((*BB)->Team == BLUE ? "BLUE" : "RED") << "] Evade Z=" << MyLocation.Z << std::endl;

	// 2026-07-12 진단: 장시간(15초+) 끊기지 않는 순수 Evade 스트릭의 원인 규명용.
	// Side(=MyRight·LOS)가 0 근처에서 진동하면 TurnSign이 매틱 뒤집혀 브레이크
	// 방향이 결정을 못 내리는 가설, ATA/Dist가 실제로 개선되고 있는지 여부를 확인.
	static int __evDiag[2] = { 0, 0 };
	if (++__evDiag[__t] % 10 == 0)
	{
		// DECO_TailThreatCheck와 동일 공식(MyToTarget.angleBetween(MyForward))으로
		// 재계산한 "진짜" ATA. BB->MyAngleOff_Degree는 이름과 달리 실제로는
		// AngleOffUpdate.cpp에서 "내 기수 vs 타겟 기수" 각(heading crossing angle)을
		// 담고 있어 TailThreat 게이트가 쓰는 값과 다름 -- 혼동 방지용으로 별도 계산.
		Vector3 MyForward = (*BB)->MyForwardVector;
		float trueAtaDeg = MyLocation.distance(TargetLocation) > 1e-3
			? (float)((TargetLocation - MyLocation).angleBetween(MyForward) * 57.2958)
			: 0.0f;
		std::cerr << "[EVADE_DIAG] team=" << (*BB)->Team
			<< " side=" << Side
			<< " turnSign=" << TurnSign
			<< " dist=" << (*BB)->Distance
			<< " trueAta=" << trueAtaDeg
			<< " headingXAngle=" << (*BB)->MyAngleOff_Degree
			<< " aa=" << (*BB)->MyAspectAngle_Degree
			<< std::endl;
	}

	return NodeStatus::SUCCESS;
}
