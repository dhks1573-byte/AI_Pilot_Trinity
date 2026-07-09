#include "Task_Pure.h"
#include <iostream>

PortsList Action::Task_Pure::providedPorts()
{
	return {
			InputPort<CPPBlackBoard*>("BB")
	};
}

NodeStatus Action::Task_Pure::tick()
{
	Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
	Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;
	Vector3 MyLocation    = (*BB)->MyLocation_Cartesian;

	double Distance = MyLocation.distance(TargetLocation);

	// 2026-07-09: 리드(요격점) 조준을 버리고 진짜 Pure Pursuit(현재 타겟 위치를
	// 그대로 조준)로 되돌림. 07-03에 Task_Lead와 공식을 통일했던 이유(914m 경계에서
	// VP가 튀는 불연속 방지)는 여전히 유효하지만, 10시드 배치 텔레메트리로 확인한
	// 결과 리드 방식으로는 WEZ 사거리(152~1219m) 안에서 실제 발사조건(ATA<1~3도)에
	// 단 한 번도 도달하지 못했다([[session-2026-07-09-aip-dogfight-ata-vs-aa]] 후속3).
	// 반면 좋은 각(ATA<2도)은 원거리(1930~2846m)에서는 실제로 잡힌다 -> 사거리 안에
	// 들어온 뒤에는 "미래 요격점을 쫓기"보다 "지금 이 순간 상대를 정확히 겨누기"가
	// 우선이라고 판단, WEZ 전용 태스크인 이 Task_Pure만 리드 계산을 제거함(Task_Lead는
	// 원거리 접근용이라 미변경). 914m 경계의 VP 불연속은 CPPBehaviorTree.cpp의
	// boresight 각도클램프(2026-07-09 신설)가 매 틱 각도 변화를 제한해주므로 완화될
	// 것으로 기대 — 배치 검증으로 실제 효과 확인 필요.
	Vector3 VP = TargetLocation;

	// 강하각 클램프 pursuit: 상승은 관대(~26.5도), 강하는 보수적(~11도)으로 비대칭 제한.
	// 바닥도 ClimbOut 트리거(3000m)보다 위로 올려서 회복 여유 확보.
	double climbSlope = Distance * 0.5;
	double diveSlope = Distance * 0.2;
	double minZ = MyLocation.Z - diveSlope;
	double maxZ = MyLocation.Z + climbSlope;

	if (VP.Z < minZ) VP.Z = minZ;
	if (VP.Z > maxZ) VP.Z = maxZ;
	if (VP.Z < 3500.0) VP.Z = 3500.0;
	(*BB)->VP_Cartesian = VP;

	// 2026-07-09: 폐쇄율(closure rate) 관리 -- 지금까지 스로틀이 항상 1.0로 고정돼
	// 있어서(RunCPPBT, BB->Throttle은 아무도 안 씀) WEZ 안에서 좋은 각을 잡아도 그대로
	// 감속 없이 상대를 지나쳐버리는 오버슈트가 반복됐다(37/18000틱만 Task_Pure 실행,
	// [[session-2026-07-09-aip-dogfight-ata-vs-aa]] 후속4). 07-03에 이미 "거리
	// 비례" 감속을 시도했다가 실패한 전례가 있음(에너지/선회성능 저하로 체류시간·
	// 명중 둘 다 악화) -> 이번엔 거리가 아니라 "상대와의 속도차"만 보고, 이미
	// 상대보다 유의미하게(20m/s+) 빠를 때만 소폭(0.7) 스로틀백 -- 선회에 필요한
	// 에너지를 크게 깎지 않으면서 폐쇄율만 낮춰 WEZ 체류시간을 늘리는 게 목표.
	float myThrottle = 1.0f;
	double speedMargin = (*BB)->MySpeed_MS - (double)(*BB)->TargetSpeed_MS;
	if (speedMargin > 20.0)
	{
		myThrottle = 0.7f;
	}
	(*BB)->Throttle = myThrottle;

	static int __dbg[2] = { 0, 0 };
	int __t = ((*BB)->Team == BLUE) ? 0 : 1;
	if (++__dbg[__t] % 30 == 0) std::cerr << "[ACTIVE] [" << ((*BB)->Team == BLUE ? "BLUE" : "RED") << "] Pure Z=" << MyLocation.Z << " VPZ=" << VP.Z << std::endl;

	// 진단용(임시): WEZ 내 실제 조준정확도(ATA) 수치 확인, 2026-07-05
	{
		Vector3 MyForward = (*BB)->MyForwardVector;
		Vector3 MyToTarget = TargetLocation - MyLocation;
		float ata = (float)(MyToTarget.angleBetween(MyForward) * 57.2958);
		std::cerr << "[PURE_ATA] team=" << (*BB)->Team << " ata=" << ata
			<< " dist=" << Distance << " leadDist=" << (VP - TargetLocation).length() << std::endl;
	}

	return NodeStatus::SUCCESS;
}
