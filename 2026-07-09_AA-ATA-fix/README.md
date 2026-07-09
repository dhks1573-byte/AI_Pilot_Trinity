# 2026-07-09 개선 작업 정리 — "0승"의 원인 규명 + 패배 0건 달성

## 한줄 요약

**BFM 판정이 "각도"를 잘못 보고 있었고(AA vs ATA 혼동), 실제로 조준을 담당하는
Controller_CY는 큰 각도에서 고착되는 결함이 있었고, 스로틀은 아예 구현된 적이
없었다.** 이 세 가지를 Controller_CY 자체는 건드리지 않고 상위 레이어(BT Task,
XML 우선순위, 스로틀 로직)에서 우회/보완해서, 20시드 배치 기준 **mean reward
-150.65 → -99.06으로 개선, 패배 6건 → 0건**을 달성했습니다. 다만 **승리는 아직
0건**이며, 왜 그런지도 원인이 규명되어 있습니다(아래 "남은 문제" 참고).

모든 변경은 `run_batch_dogfight.py`(20시드 랜덤 스폰 배치)로 회귀 검증했고,
회귀가 확인된 시도는 전부 즉시 원복했습니다. 아래 표와 각 섹션에 "성공/실패"를
명확히 구분해뒀습니다 — 실패한 시도도 이유가 명확해서 재시도 방지에 도움이 될
것 같아 그대로 남겨둡니다.

## 검증 결과 요약 (20시드 배치, Rule_BFMSelect vs Rule_base_target)

| 단계 | 변경 | mean reward | 승/패/무 | 결론 |
|---|---|---|---|---|
| 0 | (기존 상태) | -150.65 | 0/6/14 | 기준선 |
| 1 | boresight 각도클램프 (Controller_CY 우회) | -111.52 | 0/**0**/20 | ✅ 성공, 패배 전멸 |
| 2 | BFMClassifier 판정순서 재정렬 | -111.52 (동일) | 0/0/20 | ⚠️ 안전하나 이 배치에선 효과 미검증 |
| 3 | Task_Pure를 진짜 Pure Pursuit으로 | -102.92 | 0/0/20 | ✅ 소폭 개선 |
| 4 | Task_Pure 전용 속도차 스로틀(0.7배속) | **-99.06** | 0/0/20 | ✅ 소폭 개선 |
| 5a | Task_Evade에 AA 가드 추가 (시도) | -106.48 (8시드) | 0/**1**/7 | ❌ 회귀, 원복 |
| 5b | Task_Lead에도 속도차 스로틀 적용 (시도) | -115.05 | 0/0/20 | ❌ 회귀, 원복 |

`Rule_Trinity`(다른 트리)에서도 1번 변경(boresight 클램프)을 검증: -150.65 →
-94.11(한 시드는 +73.37까지 나옴), 마찬가지로 패배 0건.

---

## 배경: 왜 이 문제를 팠는가

기존에는 "0승"의 원인을 몰랐습니다. 텔레메트리(`BFMClassifier.cpp`가 30틱마다
찍는 `[BFM] team=... mode=... MyATA=... AA=... dE=... Dist=...` 로그)를 8시드
배치로 분석한 결과, 결정적인 걸 발견했습니다:

- `AA`(aspect angle, "내가 상대 꼬리를 물었는가")와 `MyATA`(antenna train angle,
  "내 기수가 실제로 상대를 겨누는가")는 **완전히 다른 각도**입니다.
- `BFMClassifier`와 전술 로직 전체는 `AA`만 보고 "공격 위치"를 판단합니다.
- 그런데 **실제 승리/데미지 조건은 오직 `MyATA`(ATA)만 봅니다**
  (`DogFightEnv/Release/src/dogfight/config.py`의 `wez.angle_deg=2.0`,
  `reward.py`의 `ata_factor`).
- 실측: "OBFM & WEZ 안(914m 이내)"이라고 분류기가 확신한 57개 순간 전부에서
  `MyATA`는 84~170도(발사조건 2도의 40~85배) — **단 한 번도 실제로 쏠 수 있는
  각도가 아니었습니다.**

이게 왜 이렇게 되는지 원인도 이미 예전 세션(07-06)에서 규명돼 있었습니다:
`Controller_CY::GetStick`의 피치 커맨드 로직이 `LOS>=90도`일 때 연속식이 아니라
`UpVector.Z` 부호만 보는 이진(±1) 커맨드로 바뀌는데, 롤이 90도/270도를 지날
때마다 이 부호가 뒤집혀서 **자기 재생산 고착(limit cycle)**이 생깁니다. 즉
위치는 잘 잡아도(`AA` 좋음) 기수를 실제로 돌리지 못하는(`MyATA` 안 줄어듦) 구조적
결함이었습니다. `Controller_CY` 자체를 3번 직접 고쳐봤지만(07-06 세션) 전부
기준선을 더 악화시켜서(가장 좁게 게이팅한 시도가 가장 나쁨) 실패했었습니다.

---

## 변경 1: boresight 각도클램프 (Controller_CY 우회) — ✅ 가장 큰 성공

**파일**: `AIP_DCS/BehaviorTree/CPPBehaviorTree.cpp` (`Step()` 함수)

**아이디어**: `Controller_CY`를 한 글자도 안 건드리고, 그 위 호출부에서 VP(조준점)
자체를 조작한다. VP가 현재 기수 기준 75도보다 크게 벗어나 있으면, 그 방향으로
한 번에 조준시키지 않고 75도 지점까지만 회전시킨 중간 조준점으로 대체한다
(`Vector3::sLerp` 사용). 매 틱 실제 기수가 갱신되며 재계산되는 폐루프라서, 기수가
돌아오는 만큼 VP가 자연히 원래 목표로 수렴한다. 이렇게 하면 `Controller_CY`는
항상 `LOS<90`인 안전한 연속식 구간에서만 동작하게 되어, 애초에 이진 고착
특이점을 만나지 않는다.

**변경 전** (`Step()` 함수, `RunCPPBT` 직후):
```cpp
	//블랙보드에 입력된 정보를 바탕으로 비헤비어트리 Run
	RunCPPBT(VP, Throttle, AimmingMode);


	R = Controller.GetStick(
		BB->MyLocation_Cartesian,
		Vector3(BB->MyRotation_EDegree.Roll * DEG2RAD,
			BB->MyRotation_EDegree.Pitch * DEG2RAD,
			BB->MyRotation_EDegree.Yaw * DEG2RAD),
		VP);
```

**변경 후**: `RunCPPBT`와 `Controller.GetStick` 사이에 아래 블록 삽입(전체 코드는
`수정된_소스코드/CPPBehaviorTree.cpp` 참고):
```cpp
	// Controller_CY의 LOS>=90도 이진 피치 고착을 Controller_CY 자체를 건드리지
	// 않고 task 레벨에서 우회한다.
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
```

또한 같은 함수의 `RunCPPBT()` 안에서 스로틀을 항상 무시하고 `1.0f`로 덮어쓰던 걸
`BB->Throttle`을 실제로 반영하도록 고쳤습니다(변경 4와 연결):
```cpp
	// 변경 전: Throttle = 1.0f;   (BB->Throttle은 완전히 무시)
	// 변경 후:
	BB->Throttle = 1.0f;  // 매틱 기본값 리셋 (tickRoot() 호출 전)
	...
	Throttle = BB->Throttle;  // 트리가 설정한 값을 실제로 반영
```

**검증**: 20시드 배치 기준 `Rule_BFMSelect`/`Rule_Trinity` 둘 다 패배 0건 달성.
두 트리에 공통 적용되는 지점이라 트리 종류와 무관하게 개선됨.

---

## 변경 2: BFMClassifier 판정 순서 재정렬

**파일**: `AIP_DCS/BehaviorTree/BT_Content/Service/BFMClassifier.cpp`

사용자가 리플레이를 보고 "교전 의지가 없어 보인다"고 지적해서 원인을 팠습니다.
`BFMClassifier`의 판정 순서가 `TAILTHREAT(MyATA>150)` → `AA<60(OBFM)` →
`ENERGY(dE<-300)` 순이었는데, 데모 텔레메트리(436샘플) 분석 결과 **DBFM(방어/이탈)
171건 중 110건(64%)이 실제로는 이미 AA<60(좋은 조준 위치)인 상태에서 발동**되고
있었습니다. 정상적인 추격 중 자연스러운 오버슈트(위치는 좋은데 순간적으로 기수만
딴 데를 보는 상황)를 "적이 내 뒤에 있어 위협"으로 오판한 것입니다.

**변경**: `AA<60` 체크를 최우선 순위로 승격(순서만 바꿈, 로직 자체는 안 바꿈):
```cpp
// 변경 전
if (InTacticalRange && MyATA > TAILTHREAT_THRESHOLD) { NewBFM = DBFM; }
else if (AA < AA_THRESHOLD) { NewBFM = OBFM; }
else if (InTacticalRange && dE < -ENERGY_THRESHOLD) { NewBFM = DBFM; }
...

// 변경 후
if (AA < AA_THRESHOLD) { NewBFM = OBFM; }  // 최우선으로 승격
else if (InTacticalRange && MyATA > TAILTHREAT_THRESHOLD) { NewBFM = DBFM; }
else if (InTacticalRange && dE < -ENERGY_THRESHOLD) { NewBFM = DBFM; }
...
```

**주의(중요)**: 20시드 배치에서는 재정렬 전/후 reward가 **bit-identical**했습니다.
원인을 diff로 추적해보니, `Rule_BFMSelect.xml`의 `Task_Evade` 시퀀스가 BFM 분류
보다 **우선순위가 높은 독립 게이트**(`DECO_TailThreatCheck ThreatAngle=150`,
BFMClassifier의 옛 TAILTHREAT 체크와 완전히 동일한 각도 공식 사용)를 갖고 있어서,
BFMClassifier가 뭐라고 판단하든 이 게이트가 먼저 가로채고 있었습니다. 즉 이번
재정렬의 실익은 이 배치 시나리오에서는 안 드러났지만(회귀는 없음), 코드 자체는
버그가 맞고 다른 시나리오에서는 도움이 될 수 있습니다.

**Task_Evade 게이트도 같은 방식으로 고쳐보려 시도했으나 실패했습니다** — 아래
"실패한 시도" 참고.

---

## 변경 3: Task_Pure를 진짜 Pure Pursuit으로 되돌림

**파일**: `AIP_DCS/BehaviorTree/BT_Content/Task/Task_Pure.cpp`

07-03 세션에 `Task_Pure`(WEZ 안 정밀조준용)가 `Task_Lead`(원거리 접근용)와 똑같은
"리드 포인트"(요격점, 상대 미래 위치) 공식을 쓰도록 통일돼 있었습니다(914m 경계에서
VP가 튀는 불연속을 없애려는 목적). 그런데 10시드 배치 텔레메트리로 확인해보니
**이 리드 방식으로는 WEZ 사거리(152~1219m) 안에서 실제 발사조건(ATA 1~3도)에
단 한 번도 도달하지 못했습니다.** 반면 좋은 각(ATA<2도)은 원거리(1930~2846m)에서는
실제로 잡힙니다.

**변경**: WEZ 전용 태스크인 `Task_Pure`만 리드 계산을 제거하고 "지금 이 순간
상대 위치"를 그대로 조준하도록 되돌림(`Task_Lead`는 원거리 접근용이라 미변경):
```cpp
// 변경 전
TargetForward.normalize();
Vector3 TargetVelocity = TargetForward * (double)(*BB)->TargetSpeed_MS;
double InterceptTime = ...;  // 리드 시간 계산
Vector3 VP = TargetLocation + TargetVelocity * InterceptTime;

// 변경 후
Vector3 VP = TargetLocation;  // 리드 없이 현재 위치 그대로
```

914m 경계의 VP 불연속 문제는 변경 1의 boresight 클램프가 매 틱 각도 변화를
제한해주므로 완화된다고 기대했고, 실제로 회귀 없이 소폭 개선됐습니다.

---

## 변경 4: Task_Pure 전용 속도차 기반 스로틀 제어

**파일**: `AIP_DCS/BehaviorTree/BT_Content/Task/Task_Pure.cpp`,
`AIP_DCS/BehaviorTree/CPPBehaviorTree.cpp`

`RunCPPBT()`를 보다가 발견: **`Throttle`이 지금까지 단 한 번도 실제로 제어된 적
없이 항상 `1.0f`로 하드코딩**돼 있었습니다(원본 주석: "쓰로틀 임시값, 개발
하면서 AI가 만들어내는 값을 넣으세요"). `BB->Throttle` 필드는 초기화(0)된 후
어떤 Task도 쓴 적 없는 완전히 죽은 값이었습니다. 이게 "좋은 각을 잡아도 감속
없이 그대로 상대를 지나쳐버리는" 오버슈트의 원인 중 하나로 보였습니다.

**주의**: 코드에 이미 07-03 날짜의 실패 기록이 남아있었습니다 — "거리 비례"
감속을 시도했다가 에너지/선회성능 저하로 체류시간·명중 모두 악화(3.7%→2.7%)돼서
원복한 전례. 이번엔 거리 대신 **속도차**로 재시도:

```cpp
// Task_Pure.cpp에 추가
float myThrottle = 1.0f;
double speedMargin = (*BB)->MySpeed_MS - (double)(*BB)->TargetSpeed_MS;
if (speedMargin > 20.0)  // 상대보다 20m/s 이상 빠를 때만
{
    myThrottle = 0.7f;  // 완전 감속이 아니라 소폭 스로틀백
}
(*BB)->Throttle = myThrottle;
```

**결과**: 20시드 중 단 2개 시드에서만 실제로 값이 달라짐(나머지 18개는 속도차
조건이 안 걸림) — 1개는 소폭 악화(-4.28), **1개는 큰 폭 개선(+81.45)**. 발동
빈도는 낮지만 비대칭적 이득이 순이익으로 이어짐.

**교훈**: 07-03의 "거리 비례" 실패와 이번 "속도차 기반" 성공을 비교하면, 폐쇄율
관리는 **거리(공간 정보)가 아니라 상대 속도(운동 정보)를 기준으로 판단해야
에너지 낭비 없이 순수하게 폐쇄율만 조절**할 수 있습니다.

---

## 실패한 시도 (원복 완료, 그래도 기록해둠)

### 5a. Task_Evade에도 AA 가드 추가 — 회귀로 원복

변경 2와 같은 원리로, `Rule_BFMSelect.xml`의 `Task_Evade` 시퀀스에도
`DECO_AspectAngleCheck UpDown="Greater" InputAA="60"` 가드를 추가해봤습니다(AA가
이미 좋으면 Evade가 못 뜨도록). 결과:
- 단일 데모: reward -173.05 → **-180.87 악화**, 이번 세션 최초로 **ownship이
  실제 피격**당함(health 1.0 → 0.857)
- 8시드 배치: mean -97.43(0패) → **-106.48(1패로 회귀)**

**즉시 원복.** 교훈: `MyATA>150`은 위치(AA)가 아무리 좋아도 "지금 당장은 최소한
방어 기동이라도 해야 하는" 유효한 신호였습니다. BFMClassifier의 "느슨한" 분류
(공격 지속 여부)와 Task_Evade의 "긴급" 게이트(생존 반응)는 같은 각도를 쓰더라도
서로 다른 기준으로 판단해야 합니다.

### 5b. Task_Lead에도 동일 스로틀 적용 — 회귀로 원복

변경 4가 잘 되자, 같은 조건(속도차>20m/s → throttle 0.7)을 원거리 접근용
`Task_Lead`에도 그대로 적용해봤습니다. 결과: 20시드 mean -99.06 → **-115.05로
악화**(패배 재발은 없었음). 즉시 원복.

**교훈**: 같은 조건, 같은 감속폭이라도 **어느 태스크/거리 구간에 적용하느냐에
따라 결과가 정반대**입니다. WEZ 근접 단계에서는 폐쇄율을 낮추는 게 이득이지만,
원거리 접근 단계에서 감속하면 접근 자체가 느려지고 에너지/시간만 낭비합니다.
"검증된 로직을 다른 곳에 그대로 재사용"이 항상 안전한 확장은 아닙니다.

---

## 남은 문제 (다음 우선순위)

**실제 발사조건(ATA<1~3도) 도달은 여전히 0/3785건**입니다. 좋은 각(ATA<2도)은
원거리(1930m+)에서는 실제로 잡히지만, 사거리(1219m 이내)에 들어오는 시점에는
소실됩니다 — 원거리에서 반짝 좋은 각을 잡았다가 계속 접근하며 각이 다시 벌어지는
오버슈트 패턴입니다. 다음 세션은 **원거리에서 잡힌 좋은 각을 사거리 진입 시점까지
유지시키는 방법**을 정면으로 다뤄야 합니다.

## 대회 공식 규정 관련 중요 발견 (별도 조사)

대회 디스코드 질문사항을 확인하다가, **로컬 검증 환경이 실제 대회 규칙과
다르다**는 걸 발견했습니다:
- 실제 데미지는 **Phase1/2/3 nested WEZ**(시간에 따라 ATA<1°/2°/3°,
  3000/3500/4000ft로 조건이 완화됨)인데, 로컬 `config.py`/`reward.py`는 Phase1과
  비슷한 **단일 WEZ**(ATA≤2도, 500~3000ft)만 구현돼 있습니다.
- 라운드 길이도 실제는 **200초**인데 로컬 기본값은 **300초**입니다.
- **무한 회피 무승부 규정**(양쪽 다 30초 이상 교전 의지 없이 회피만 하면 이후
  30초 내 교전 안 되면 무승부)도 확인했습니다 — 오늘 고친 "좋은 위치에서도
  이탈해버리는" 버그가 실전에서 이 규정에 직접 걸릴 수 있는 문제였다는 뜻입니다.

이 부분은 팀 차원에서 로컬 환경에 Phase 시스템을 반영할지 논의가 필요해 보입니다.

## 빌드/검증 방법 (재현하고 싶은 경우)

```powershell
# 빌드
& "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" `
  "AIP_DCS_2026_05_26\AIP_DCS\AIP_DCS.sln" -p:Configuration=Release -p:Platform=x64

# 배포 (빌드 산출물을 DogFightEnv\Release\AIP_DCS_ownship.dll로 복사)

# 20시드 배치 검증
cd DogFightEnv\Release
python run_batch_dogfight.py --ownship-bt-dll AIP_DCS_ownship.dll `
  --ownship-bt-rule-xml Rule_BFMSelect.xml --target-bt-dll AIP_DCS_target.dll `
  --target-bt-rule-xml Rule_base_target.xml --num-seeds 20 --quiet
```

**주의**: 배치를 두 개 이상 동시에 실행하면 안 됩니다 — 룰 XML을 DLL 옆 공유
파일로 복사하는 방식이라 서로 덮어씁니다. 항상 순차 실행하세요.

## 첨부 파일 (`수정된_소스코드/` 폴더)

- `CPPBehaviorTree.cpp` — boresight 클램프 + 스로틀 반영 (변경 1)
- `BFMClassifier.cpp` — 판정 순서 재정렬 (변경 2)
- `Task_Pure.cpp` — Pure Pursuit 전환 + 스로틀 제어 (변경 3, 4)
- `Task_Lead.cpp` — 최종적으로 **원본과 동일**(스로틀 적용 시도했다가 원복,
  참고용으로 첨부)
- `Rule_BFMSelect.xml` — Task_Evade 시도 기록이 주석으로 남아있음(로직 자체는
  원본과 동일)
