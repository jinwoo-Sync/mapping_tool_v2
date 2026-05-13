# Voxel-SLAM vs Fast-LIO2 알고리즘 차이 분석 보고서

## 문제 개요

### 발생한 문제
- **시스템**: Voxel-SLAM with HDL32E LiDAR
- **증상**: 직선 구간에서는 정상 작동하지만 곡선/회전 구간에서 odometry 실패
- **에러 메시지**: eigenvalue < 14로 인한 LIO 실패, 로봇이 후진하는 것처럼 표시됨
- **하드웨어**: HDL32E LiDAR 뒤쪽 향함 + 35° 하향 틸트, IMU 수평 장착

### 비교 대상
- **Fast-LIO2**: 동일한 하드웨어 설정에서 정상 작동
- **의문점**: 왜 Voxel-SLAM은 실패하고 Fast-LIO2는 성공하는가?

## 핵심 발견사항

### 1. Eigenvalue 기반 Observability 검증의 차이

#### Voxel-SLAM (매우 엄격한 검증)
```cpp
// voxel_map.hpp:1028-1031
bool plane_judge(Eigen::Vector3d &eig_values) {
    return (eig_values[0] < min_eigen_value && 
            (eig_values[0]/eig_values[2]) < plane_eigen_value_thre[layer]);
}

// voxel_map.hpp:1326 - 추가 엄격한 조건
if(eig_value[0]/eig_value[1] > 0.12) return; // 매우 엄격한 임계값!
```

**문제점**:
- 곡선 구간에서 뒤쪽 향하는 LiDAR의 feature 분포가 제한됨
- eigenvalue ratio가 0.12를 초과하여 연속적인 feature rejection 발생
- 결과적으로 odometry 실패

#### Fast-LIO2 (유연한 처리)
- ikd-Tree 기반으로 strict eigenvalue threshold 없음
- Continuous feature matching with graceful degradation
- Temporary poor observability를 tolerant하게 처리

### 2. Map Representation 차이점

#### Voxel-SLAM
- **구조**: OctoTree 기반 계층적 voxel map
- **특징**: 각 voxel에서 독립적인 plane fitting
- **문제**: Poor observability 시 entire voxel rejection

#### Fast-LIO2
- **구조**: ikd-Tree (동적으로 balance되는 k-d tree)
- **특징**: 지속적인 incremental update
- **장점**: Local poor observability를 global consistency로 보상

### 3. 좌표 변환 처리 방식

#### Voxel-SLAM
```cpp
// 복잡한 multi-step coordinate transformation
Eigen::Vector3d P_compensate = xc.R.transpose() * 
    (R_i * (extrin_para.R * P_i + extrin_para.p) + T_ei);
```
- 엄격한 순서의 좌표 변환
- 각 단계에서 eigenvalue validation 수행

#### Fast-LIO2
- 더 robust한 coordinate frame handling
- Backward-forward propagation으로 error accumulation 방지

## 기하학적 분석

### 뒤쪽 LiDAR + 곡선 구간의 문제

**하드웨어 설정**:
- HDL32E LiDAR: 후방 향함, 35° 하향 틸트
- Extrinsic matrix 분석 결과: 실제 물리적 배치와 일치

**곡선 구간에서의 기하학적 제약**:
1. 차량이 좌/우 회전할 때 뒤쪽 LiDAR가 관측하는 feature들의 기하학적 분포가 매우 제한됨
2. 주로 지면과 일부 구조물만 관측 가능
3. Feature diversity 부족으로 eigenvalue degradation 발생

## 해결 방안 적용

### 적용된 Configuration 변경

#### hdl32e.yaml 수정사항
```yaml
# Odometry 섹션
min_eigen_value: 0.001  # 0.002 → 0.001로 완화

# LocalBA 섹션  
plane_eigen_value_thre: [8.0, 8.0, 8.0, 8.0]  # [3.0, 3.0, 3.0, 3.0] → 완화
```

**변경 이유**:
- 뒤쪽 LiDAR의 제한된 관측 조건에 맞춰 eigenvalue threshold 완화
- Fast-LIO2와 유사한 level의 observability tolerance 제공

## 알고리즘 비교 요약

| 특성 | Voxel-SLAM | Fast-LIO2 |
|------|------------|-----------|
| **Observability 검증** | 매우 엄격한 eigenvalue threshold | 유연한 feature matching |
| **Map 표현** | 계층적 voxel (rigid structure) | ikd-Tree (adaptive structure) |
| **Poor observability 처리** | Complete rejection | Graceful degradation |
| **좌표 변환** | Multi-step validation | Robust propagation |
| **Feature matching** | Plane-based (strict geometry) | Point-to-plane (flexible) |
| **메모리 효율성** | 높음 (voxel 기반) | 중간 (tree 기반) |
| **실시간 성능** | 빠름 (fixed structure) | 중간 (dynamic balancing) |

## 근본적 설계 철학 차이

### Voxel-SLAM
- **철학**: "Precision over Robustness"
- **접근**: 엄격한 feature validation으로 정확도 우선
- **결과**: 이상적 조건에서 높은 정확도, 제한된 조건에서 실패 가능

### Fast-LIO2  
- **철학**: "Robustness over Precision"
- **접근**: 유연한 feature matching으로 안정성 우선
- **결과**: 다양한 조건에서 안정적 동작, 약간의 정확도 trade-off

## 추가 개선 방향

### 1. 단기 개선 (Configuration 기반)
- ✅ **완료**: eigenvalue threshold 완화
- **추가 고려사항**: voxel_size 조정 (현재 2.5m → 상황에 따라 조정)

### 2. 중기 개선 (알고리즘 수정)
```cpp
// 제안: voxel_map.hpp의 엄격한 조건 완화
if(eig_value[0]/eig_value[1] > 0.20) return; // 0.12 → 0.20으로 완화
```

### 3. 장기 개선 (하이브리드 접근)
- Voxel-SLAM의 효율성 + Fast-LIO2의 robustness 결합
- Dynamic threshold adjustment 기반 adaptive observability check

## 결론

**핵심 원인**: Voxel-SLAM의 과도하게 엄격한 observability 검증이 뒤쪽 LiDAR의 제한된 관측 조건과 결합되어 곡선 구간에서 연속적인 feature rejection 발생

**해결책**: Configuration parameter 조정을 통해 Fast-LIO2 수준의 observability tolerance 제공

**교훈**: 실제 하드웨어 배치와 운용 환경을 고려한 algorithm parameter tuning의 중요성

---

**작성일**: 2025-08-21  
**분석 대상**: Voxel-SLAM vs Fast-LIO2  
**하드웨어**: HDL32E (rear-facing, 35° downward tilt) + IMU