// Fill out your copyright notice in the Description page of Project Settings.


#include "CPPBehaviorTree.h"

#include <exception>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <functional>


Vector3 UCPPBehaviorTree::LLAtoCartesian(Vector3 LLA, Vector3 BaseLLA)
{
	double eccentricitysquare, N, M;
	eccentricitysquare = 1.0 - pow(6356752.3142, 2) / pow(6378137.0, 2);
	N = 6378137.0 / sqrt(1.0 - eccentricitysquare * pow(sin(BaseLLA.X * PI / 180.0), 2)); // prime vertical radius of curvature
	M = 6378137.0 * (1.0 - eccentricitysquare) / pow(1 - eccentricitysquare * pow(sin(BaseLLA.X * PI / 180.0), 2), 3 / 2);

	double dlat, dlon;
	dlat = LLA.X - BaseLLA.X;
	dlon = LLA.Y - BaseLLA.Y;

	double dN, dE, dD;
	dN = (M + BaseLLA.Z) * dlat * PI / 180.0;
	dE = (N + BaseLLA.Z) * cos(BaseLLA.X * PI / 180.0) * dlon * PI / 180.0;
	dD = (LLA.Z - BaseLLA.Z);
	Vector3 res(dN, dE, dD);
	return res;
}

// Sets default values for this component's properties
UCPPBehaviorTree::UCPPBehaviorTree()
{
	ID = -1;
	ForceID = -1;

	f2m = 3.28084;
	EQ_R = 6.378137E+6;
	P_R = 6.3567523142E+6;
	fr = 298.257223563;
	Req = 6.378137E+6;
	d2r = 3.1415926535897931 / 180.0;
	m2f = 3.28084;


	elev0 = 0.2;
	aile0 = 0.0;
	eccen = 1.0 - P_R * P_R / (EQ_R * EQ_R);
	bInitialized = false;

	BB = new CPPBlackBoard();
}


UCPPBehaviorTree::~UCPPBehaviorTree()
{
	delete BB;
}


void UCPPBehaviorTree::init()
{
	bInitialized = false;

	try
	{
		/*
		노드 입력 : 구현해둔 노드들을 Factory 객체에 입력해주는 과정
		
		새로 생성한 노드를 여기에 입력해주세요!!!!!!
		*/
		Factory.registerNodeType<Action::SelectTarget>("SelectTarget");
		Factory.registerNodeType<Action::DistanceUpdate>("DistanceUpdate");
		Factory.registerNodeType<Action::CheckSight>("CheckSight");
		Factory.registerNodeType<Action::AngleOffUpdate>("AngleOffUpdate");
		Factory.registerNodeType<Action::DirectionVectorUpdate>("DirectionVectorUpdate");
		Factory.registerNodeType<Action::AspectAngleUpdate>("AspectAngleUpdate");
		Factory.registerNodeType<Action::BFMClassifier>("BFMClassifier");
		Factory.registerNodeType<Action::DECO_BFMCheck>("DECO_BFMCheck");
		Factory.registerNodeType<Action::DECO_DistanceCheck>("DECO_DistanceCheck");
		Factory.registerNodeType<Action::DECO_AngleOffCheck>("DECO_AngleOffCheck");
		Factory.registerNodeType<Action::DECO_AspectAngleCheck>("DECO_AspectAngleCheck");
		Factory.registerNodeType<Action::Task_Empty>("Task_Empty");
		Factory.registerNodeType<Action::Task_Pure>("Task_Pure");
		Factory.registerNodeType<Action::Task_Lead>("Task_Lead");
		Factory.registerNodeType<Action::Task_Evade>("Task_Evade");
		Factory.registerNodeType<Action::Task_ClimbOut>("Task_ClimbOut");
		Factory.registerNodeType<Action::Task_LevelOff>("Task_LevelOff");
		Factory.registerNodeType<Action::Task_BreakAndReverse>("Task_BreakAndReverse");
		Factory.registerNodeType<Action::Task_LoopAttack>("Task_LoopAttack");
		Factory.registerNodeType<Action::Task_GetTail>("Task_GetTail");
		Factory.registerNodeType<Action::Task_Engage>("Task_Engage");
		Factory.registerNodeType<Action::Task_TriCategoryBlend>("Task_TriCategoryBlend");
		Factory.registerNodeType<Action::Task_Merge>("Task_Merge");
		Factory.registerNodeType<Action::Task_Extend>("Task_Extend");
		Factory.registerNodeType<Action::Task_PurePN>("Task_PurePN");
		Factory.registerNodeType<Action::DECO_AltitudeCheck>("DECO_AltitudeCheck");
		Factory.registerNodeType<Action::DECO_MaxAltitudeCheck>("DECO_MaxAltitudeCheck");
		Factory.registerNodeType<Action::DECO_TailThreatCheck>("DECO_TailThreatCheck");



		//파일로 트리 구조 정의
		//자신의 팀 이름으로	xml 파일 만들어서 입력해주세요!!!!!! (Rule_forTraining.xml은 예시입니다)
		tree = Factory.createTreeFromFile("./Rule_forTraining.xml");


		//블랙보드 연결 : 원래는 블랙보드 내에 있는 모든 변수를 하나하나 이런식으로 입력해줘야하는 미친 비효율을 보이는 방식이지만 커스텀 블랙보드를 만들어 해당 블랙보드를 입력시킴
		tree.rootBlackboard()->set<CPPBlackBoard*>("BB", BB);
		
		bInitialized = true;
		std::cout << "Behavior Tree Initialized Successfully" << std::endl;
	}
	catch (const std::exception& e)
	{

		std::cout << "Behavior Tree Initialization Failed: " << e.what() << std::endl;

		std::cout << "It appears that the process failed while parsing the XML." << std::endl;
		std::cout << " -Please check whether the XML file is located in the correct path." << std::endl;
		std::cout << " -Please check whether the XML file is calling any node with an invalid or incorrect name." << std::endl;
		std::cout << " -Please check whether the node was added to the Factory when building the DLL." << std::endl;
		throw;
	}
	
}

bool UCPPBehaviorTree::IsInitialized() const
{
	return bInitialized;
}

StickValue UCPPBehaviorTree::Step(PlaneInfo MyInfo, int NumofOtherPlane, PlaneInfo* OthersInfo, Vector3& VP, float& Throttle)
{
	PlaneInfo Myinfo;
	Myinfo.Location = MyInfo.Location;
	Myinfo.Rotation = EulerAngle(MyInfo.Rotation.Yaw, MyInfo.Rotation.Pitch, MyInfo.Rotation.Roll);
	Myinfo.AngleAcceleration = MyInfo.AngleAcceleration;
	Myinfo.Speed = MyInfo.Speed;
	Myinfo.Team = MyInfo.Team;
	Myinfo.Resv0 = MyInfo.Resv0;		//ID
	Myinfo.Resv1 = MyInfo.Resv1;		//HP
	Myinfo.Resv2 = MyInfo.Resv2;		//OperationMode

	// Convert LLA to NEU Cartesian (meters) for consistent coordinate frame with Controller
	Vector3 origin(OriLAT, OriLOn, 0.0);
	Vector3 MyLocationNEU = LLAtoCartesian(MyInfo.Location, origin);

	//다른 비행기들 위치 좌표계 변환
	PlaneInfo others[4];
	for (int i = 0; i < NumofOtherPlane; i++)
	{
		others[i].Location = LLAtoCartesian(OthersInfo[i].Location, origin);
		others[i].Rotation = EulerAngle(OthersInfo[i].Rotation.Yaw, OthersInfo[i].Rotation.Pitch, OthersInfo[i].Rotation.Roll);
		others[i].Speed = OthersInfo[i].Speed;
		others[i].Team = OthersInfo[i].Team;
		others[i].Resv0 = OthersInfo[i].Resv0;
		others[i].Resv1 = OthersInfo[i].Resv1;
		others[i].Resv2 = OthersInfo[i].Resv2;
	}

	//블랙보드의 아군기, 적군기 List 초기화
	BB->Friendly.clear();
	BB->Enemy.clear();

	//블랙보드에 내 정보(위치, 자세, 속력, 팀) 업데이트
	BB->MyLocation_Cartesian = MyLocationNEU;
	BB->MyRotation_EDegree = EulerAngle(Myinfo.Rotation.Yaw, Myinfo.Rotation.Pitch, Myinfo.Rotation.Roll);
	BB->MyAngleAcceleration = Myinfo.AngleAcceleration;
	BB->MySpeed_MS = Myinfo.Speed;
	BB->Team = (TeamColor)Myinfo.Team;

	//아군기 리스트에 내 정보 추가. Friendly의 index 0번은 무조건 나 자신
	BB->Friendly.push_back(Myinfo);

	//생존중인 비행기들의 적아 구분
	for (int i = 0; i < NumofOtherPlane; i++)
	{
		if (others[i].Resv1 > 0)
		{
			if (others[i].Team == Myinfo.Team)
			{
				BB->Friendly.push_back(others[i]);
			}
			else
			{
				BB->Enemy.push_back(others[i]);
			}
		}
		else
		{

		}
	}


	bool AimmingMode;

	StickValue R;

	//블랙보드에 입력된 정보를 바탕으로 비헤비어트리 Run
	RunCPPBT(VP, Throttle, AimmingMode);

	// Controller_CY의 LOS>=90도 이진 피치 고착(2026-07-06 세션에 텔레메트리로 확정,
	// 근본 수정 3연속 시도 전부 기준선 악화로 실패)을 Controller_CY 자체를 건드리지
	// 않고 task 레벨에서 우회한다: VP가 현재 기수 기준 MAX_OFFBORESIGHT_DEG보다 크게
	// 벗어나 있으면 그 방향으로 한번에 조준시키지 않고, 기수 기준 각도를 제한한 중간
	// 조준점으로 대체해 Controller_CY가 항상 LOS<90 연속식 구간에서만 동작하게 한다.
	// 매 틱 실제 기수(ForwardVector)가 갱신되며 다시 계산되는 폐루프라, 기수가 돌아오는
	// 만큼 자연스럽게 VP가 원래 목표 방향으로 수렴한다. Controller_CY.cpp/.h는 완전히
	// 미변경(2026-07-09, [[session-2026-07-09-aip-dogfight-ata-vs-aa]] 우회 시도).
	{
		const double MAX_OFFBORESIGHT_DEG = 75.0;

		EulerAngle EA;
		EA.Roll = BB->MyRotation_EDegree.Roll * DEG2RAD;
		EA.Pitch = BB->MyRotation_EDegree.Pitch * DEG2RAD;
		EA.Yaw = BB->MyRotation_EDegree.Yaw * DEG2RAD;
		Quaternion QU = EA.toQuaternion();

		Vector3 ForwardVector;
		ForwardVector.X = 1 - 2 * (QU.X * QU.X + QU.Y * QU.Y);
		ForwardVector.Y = 2 * (QU.X * QU.Z + QU.W * QU.Y);
		ForwardVector.Z = -2 * (QU.Y * QU.Z - QU.W * QU.X);
		ForwardVector.normalize();

		Vector3 RawDir = VP - BB->MyLocation_Cartesian;
		double RawDist = RawDir.length();

		if (RawDist > 1e-3)
		{
			Vector3 DirUnit = RawDir;
			DirUnit.normalize();

			double OffBoresightDeg = ForwardVector.angleBetween(DirUnit) * RADTODEG;

			if (OffBoresightDeg > MAX_OFFBORESIGHT_DEG)
			{
				double Factor = MAX_OFFBORESIGHT_DEG / OffBoresightDeg;
				Vector3 ClampedDir;
				ClampedDir.sLerp(ForwardVector, DirUnit, Factor);
				ClampedDir.normalize();
				VP = BB->MyLocation_Cartesian + ClampedDir * RawDist;
			}
		}
	}

	R = Controller.GetStick(
		BB->MyLocation_Cartesian,
		Vector3(BB->MyRotation_EDegree.Roll * DEG2RAD,
			BB->MyRotation_EDegree.Pitch * DEG2RAD,
			BB->MyRotation_EDegree.Yaw * DEG2RAD),
		VP);

	return R;

}

Vector3 UCPPBehaviorTree::GetVP()
{
	Vector3 Vp = (*BB).VP_Cartesian;
	return Vp;
}



 void UCPPBehaviorTree::RunCPPBT(Vector3& VP, float& Throttle, bool& AimmingMode)
{
	
	BB->RunningTime += BB->DeltaSecond;	//시뮬레이선 타임에 따른 델타 타임 설정

	// 2026-07-09: 매 틱 기본값을 최대로 리셋 -- 대부분의 Task는 스로틀을 신경 안 쓰므로
	// 이전 틱에 Task_Pure 등이 낮춰둔 값이 그대로 남아있지 않도록 항상 여기서 초기화.
	BB->Throttle = 1.0f;

	try
	{
		BT::NodeStatus __rootStatus = tree.tickRoot(); //트리 작동
		VP = BB->VP_Cartesian;	// VP 값

		// WEZ 안에서 거리비례로 감속시켜 체류시간을 늘려보려 했으나 실패
		// (에너지/속도가 깎여서 선회 성능이 떨어지고 오히려 WEZ 체류시간·명중 모두
		// 악화됨: 3.7%->2.7%, target_health 무피해로 후퇴) -> 원복, 2026-07-03.
		// 2026-07-09: BB->Throttle을 실제로 반영하도록 재시도(이번엔 거리가 아니라
		// Task_Pure가 상대와의 속도차만 보고 판단, [[session-2026-07-09-aip-dogfight-ata-vs-aa]]
		// 참고). 스로틀을 신경 안 쓰는 다른 모든 Task는 위에서 리셋한 기본값 1.0f를
		// 그대로 쓰게 됨.
		Throttle = BB->Throttle;

		static int __dbgRoot[2] = { 0, 0 };
		int __tRoot = (BB->Team == BLUE) ? 0 : 1;
		if (++__dbgRoot[__tRoot] % 30 == 0)
		{
			std::cerr << "[ROOT] team=" << (int)BB->Team
				<< " status=" << (int)__rootStatus
				<< " Z=" << BB->MyLocation_Cartesian.Z
				<< " VP=(" << VP.X << "," << VP.Y << "," << VP.Z << ")"
				<< std::endl;
		}
	}
	catch (const std::exception& e)
	{
		//원인을 알 수 없는 예외가 발생할 경우 VP를 (0,0,0)으로 설정하고 Throttle을 1로 설정하여 일단 최대한 안전하게 행동하도록 설정
		VP = Vector3(0,0,0);	// VP 값
		Throttle = 1.0f;	//

		std::cout << "ERROR!!!!!!!!! Behavior Tree Execution Failed: " << e.what() << std::endl;
		std::cout << "Temp Result VP : (0,0,0), Throttle : 1" << std::endl;
		
		throw;
	}

	

	
}

 void UCPPBehaviorTree::SetDeltaTime(double DT)
 {
	 BB->DeltaSecond = DT;
 }

