#include "Task_BreakAndReverse.h"
#include <cmath>
#include <algorithm>
#include <iostream>

PortsList Action::Task_BreakAndReverse::providedPorts()
{
	return { InputPort<CPPBlackBoard*>("BB") };
}

NodeStatus Action::Task_BreakAndReverse::onStart()
{
	auto BB = getInput<CPPBlackBoard*>("BB");

	Vector3 MyLocation     = (*BB)->MyLocation_Cartesian;
	Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;
	Vector3 MyForward      = (*BB)->MyForwardVector;

	// Threat check: Red almost dead astern (ATA > 140°) AND within 900m
	// (raised from 110°/1200m so we don't bail into defense on every marginal threat -- 교전 가중치 상향)
	float currentDist = (float)MyLocation.distance(TargetLocation);
	if (currentDist > 900.0f)
		return NodeStatus::FAILURE;

	Vector3 LOSToTarget = TargetLocation - MyLocation;
	LOSToTarget.normalize();
	double dot      = std::max(-1.0, std::min(1.0, MyForward.dot(LOSToTarget)));
	float  LOSAngle = (float)(std::acos(dot) * 57.2958);
	if (LOSAngle <= 140.0f)
		return NodeStatus::FAILURE;

	_phase              = Phase::BREAK;
	_breakTicks         = 0;
	_prevDistance       = -1.0f;
	_breakStartDistance = currentDist;
	return NodeStatus::RUNNING;
}

NodeStatus Action::Task_BreakAndReverse::onRunning()
{
	auto BB = getInput<CPPBlackBoard*>("BB");

	Vector3 MyLocation     = (*BB)->MyLocation_Cartesian;
	Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;
	Vector3 MyRight        = (*BB)->MyRightVector;
	Vector3 MyForward      = (*BB)->MyForwardVector;
	Vector3 TargetForward  = (*BB)->TargetForwardVector;

	float currentDist = (float)MyLocation.distance(TargetLocation);

	if (_phase == Phase::BREAK)
	{
		_breakTicks++;

		// Hard horizontal break INTO the threat (toward target's side)
		Vector3 LOSToTarget = TargetLocation - MyLocation;
		LOSToTarget.normalize();
		double side     = MyRight.dot(LOSToTarget);
		double turnSign = (side >= 0.0) ? 1.0 : -1.0;

		Vector3 breakDir = MyRight * turnSign;
		breakDir.normalize();
		(*BB)->VP_Cartesian = MyLocation + breakDir * 3000.0;

		static int __dbg1[2] = { 0, 0 };
		int __t1 = ((*BB)->Team == BLUE) ? 0 : 1;
		if (++__dbg1[__t1] % 30 == 0) std::cerr << "[ACTIVE] [" << ((*BB)->Team == BLUE ? "BLUE" : "RED") << "] BreakAndReverse-BREAK Z=" << MyLocation.Z << std::endl;

		// Overshoot detection: 틱당 거리 변화(60Hz에서 보통 수~십여 m)로 비교하면
		// 노이즈에 묻혀 거의 발동을 안 해서 BREAK에 영원히 갇히는 버그가 있었음
		// (초반 교전 이후 계속 방어 기동만 반복하며 표적과 멀어지기만 하는 원인이었음).
		// break 시작 시점 거리 대비 누적 증가로 비교하고, 그래도 못 빠져나오면
		// BREAK_TIMEOUT으로 강제 전환하는 안전장치를 둔다.
		bool overshoot = (currentDist > _breakStartDistance + 30.0f);
		if ((overshoot && _breakTicks >= BREAK_DURATION) || _breakTicks >= BREAK_TIMEOUT)
		{
			_phase = Phase::REVERSE;
		}
	}
	else // REVERSE
	{
		// Aim for Red's 6 o'clock (800m behind Red)
		TargetForward.normalize();
		Vector3 redSix = TargetLocation - TargetForward * 800.0;

		// 강하각 클램프: 이 단계는 Z를 전혀 안 건드려서 타겟이 급강하 중이면
		// 회복 여유 없이 그대로 따라 박는 사고가 있었음 (Task_Lead/Pure와 동일한 정책 적용)
		double distForClamp = MyLocation.distance(TargetLocation);
		double climbSlope = distForClamp * 0.5;
		double diveSlope = distForClamp * 0.2;
		double minZ = MyLocation.Z - diveSlope;
		double maxZ = MyLocation.Z + climbSlope;
		if (redSix.Z < minZ) redSix.Z = minZ;
		if (redSix.Z > maxZ) redSix.Z = maxZ;
		if (redSix.Z < 3500.0) redSix.Z = 3500.0;

		(*BB)->VP_Cartesian = redSix;

		static int __dbg2[2] = { 0, 0 };
		int __t2 = ((*BB)->Team == BLUE) ? 0 : 1;
		if (++__dbg2[__t2] % 30 == 0) std::cerr << "[ACTIVE] [" << ((*BB)->Team == BLUE ? "BLUE" : "RED") << "] BreakAndReverse-REVERSE Z=" << MyLocation.Z << " VPZ=" << redSix.Z << std::endl;

		// Exit: Red is in our front hemisphere → hand off to Lead/Pure
		Vector3 LOSToTarget = TargetLocation - MyLocation;
		LOSToTarget.normalize();
		double dot      = std::max(-1.0, std::min(1.0, MyForward.dot(LOSToTarget)));
		float  LOSAngle = (float)(std::acos(dot) * 57.2958);

		// 2026-07-12 진단: REVERSE 단계가 장시간(15초+) 못 빠져나오는 원인 규명용.
		// LOSAngle이 실제로 90도 밑으로 수렴해가고 있는지, 아니면 특정 값 근처에서
		// 고착/진동하는지 확인 (Controller_CY의 LOS>=90도 한계와 연관 가설).
		static int __revDiag[2] = { 0, 0 };
		if (++__revDiag[__t2] % 10 == 0)
		{
			std::cerr << "[REVERSE_DIAG] team=" << (*BB)->Team
				<< " losAngle=" << LOSAngle
				<< " dist=" << currentDist
				<< " breakTicks=" << _breakTicks
				<< std::endl;
		}

		if (LOSAngle < 90.0f)
		{
			return NodeStatus::SUCCESS;
		}
	}

	_prevDistance = currentDist;
	return NodeStatus::RUNNING;
}

void Action::Task_BreakAndReverse::onHalted()
{
	_phase        = Phase::BREAK;
	_breakTicks   = 0;
	_prevDistance = -1.0f;
}
