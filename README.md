# ConstraintBasedMotionEdit

실시간 Constraint-Based 모션 편집 — **Windows (Visual Studio 2022)**, **GLEW + GLFW + Dear ImGui + Eigen**

> Based on *"Retargetting Motion to New Characters"* — Gleicher et al.

---

## 빌드 & 실행

### 요구 사항
- **Visual Studio 2022** (Community 무료)
  - "Desktop development with C++" 워크로드 설치 필요
- 그 외 의존성(GLEW, GLFW, GLM, Eigen, Dear ImGui)은 모두 레포에 포함

### 단계별 설정

**1. 솔루션 열기**
`ConstraintBasedMotionEdit.sln`을 Visual Studio로 열기

**2. 구성 선택**
상단 툴바에서 `Debug` 또는 `Release` / `x64` 선택

> ⚠️ **반드시 x64** — x86(Win32)은 지원하지 않음

**3. 빌드 & 실행**
- `Ctrl+Shift+B` → 빌드
- `F5` / `Ctrl+F5` → 실행 (워킹 디렉터리가 자동으로 솔루션 루트로 설정됨)

**4. BVH 파일 로드**
- `.bvh` 파일을 창에 드래그 앤 드롭

---

## 조작법

| 입력 | 동작 |
|---|---|
| 마우스 드래그 (빈 곳) | 카메라 회전 |
| 마우스 스크롤 | 카메라 줌 |
| 관절 클릭 + 드래그 | IK로 관절 이동 |
| Space | 애니메이션 켜기/끄기 |
| 0 | 초기화 |
| 1 | Constraint 모션 편집 적용 |
| .bvh 드래그 앤 드롭 | BVH 파일 로드 |

---

## 구현 개요

### Inverse Kinematics
- Jacobian 기반 반복 IK 솔버 (100회 반복)
- 자코비안: `J(i,j) = axis_j × (p_end - p_joint_i)`
- SVD pseudo-inverse로 `dθ = J⁺ · dP` 풀기
- Exponential map으로 부드러운 quaternion 보간

### Constraint-Based Motion Editing
1. IK로 특정 프레임의 관절 위치 편집 → displacement 저장
2. 편집된 프레임들을 constraint로 표시
3. Cubic Uniform B-spline (knot interval = 5)으로 displacement 피팅
4. SVD로 제어점 계산: `b = p · B⁺`
5. 전체 프레임에 B-spline 커브 적용

---

## 프로젝트 구조

```
src/
  main.cpp          GLFW 윈도우, 콜백, 메인 루프, 모션 편집 로직
  IK.h/.cpp         Link/Body 데이터 구조 + IK 솔버
  BVH.h/.cpp        BVH 파서 + 포즈 적용
  Renderer.h/.cpp   카메라, 그림자 렌더링, unproject
  ShaderUtils.h/.cpp 셰이더 로드, 유니폼, 기본 도형
Res/
  shader.vert/.frag 메인 렌더링 셰이더 (조명 + 그림자)
  const.vert/.frag  단색 셰이더 (그림자 패스, 와이어프레임)
include/            GLEW, GLFW, GLM, Eigen 헤더
lib/                glew32s.lib, glfw3.lib (x64 정적)
third_party/imgui/  Dear ImGui 소스
```
