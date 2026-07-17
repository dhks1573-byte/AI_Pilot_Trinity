# 빌드된 DLL

`AIP_DCS_ownship.dll` — `수정된_소스코드/` 폴더의 변경사항(AA 부호 반전 수정,
Task_Evade 방향떨림 hysteresis 수정)이 전부 반영된 Release x64 빌드입니다.
직접 빌드하지 않고 바로 시뮬레이션에 사용하려면 `DogFightEnv/Release/AIP_DCS_ownship.dll`
자리에 이 파일로 교체하세요.

20시드 배치 검증 결과: `Rule_BFMSelect` mean reward -92.24, `Rule_Trinity` mean
reward -94.31, 둘 다 0승/0패/20무 (자세한 내용은 상위 폴더의 `README.md` 참고).
