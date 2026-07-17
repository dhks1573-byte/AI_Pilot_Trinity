//AspectAngle 업데이트 하는 서비스 노드

#include "AspectAngleUpdate.h"

namespace Action
{
	PortsList AspectAngleUpdate::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB")
		};
	}

	NodeStatus AspectAngleUpdate::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

		Vector3 MyLocation = (*BB)->MyLocation_Cartesian;				//내 위치
		Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;		//타겟 위치
		Vector3 TFV = (*BB)->TargetForwardVector;						//타겟의 Forward Vector
		Vector3 TUV = (*BB)->TargetUpVector;							//타겟의 Up Vector

		Vector3 TargetToMyPlane = MyLocation - TargetLocation;			//타겟위치에서 내위치까지의 벡터
		float P = TargetToMyPlane.dot(TUV);								//Up Vector방향 길이 구하기
		Vector3 Proj_MyLocation = MyLocation - P * TUV;					//내 위치를 적기의 Up 벡터를 법선 벡터를 가지고 적기의 위치를 지나는 평면으로 프로젝션

		Vector3 TPM = Proj_MyLocation - TargetLocation;					//프로젝션된 내 위치와 타겟사이의 벡터

		// 2026-07-11 수정: 기존 코드는 TFV(타겟의 전방벡터)와 TPM(타겟->나 벡터)을
		// 그대로 비교해서, "내가 타겟의 기수 방향(정면)에 있을 때 AA=0"이 되고 있었다.
		// 이 파일의 원래 주석("AA: 내가 타겟의 꼬리를 얼마나 잘 물었는지")과 표준 BFM
		// 관례(AA=0 -> dead six, AA=180 -> head-on) 둘 다에 정반대로 뒤집혀 있던 것.
		// BFMClassifier/DECO_AspectAngleCheck 등 이 값을 쓰는 모든 소비자가 일관되게
		// "AA가 작을수록 좋은 위치(상대 뒤)"라고 가정하고 있었으므로, 소스(TFV 부호
		// 반전)에서 한 번에 고쳐서 모든 소비자가 동시에 올바른 의미를 갖도록 함.
		float AA = TPM.angleBetween(TFV * -1.0f);							//두 벡터 사이의 각 구하기 -> AA (0=dead six, 180=head-on)

		(*BB)->MyAspectAngle_Degree = AA* 57.2958;						//디그리 값으로 사용하기 위하여 변환

		return NodeStatus::SUCCESS;
	}

}