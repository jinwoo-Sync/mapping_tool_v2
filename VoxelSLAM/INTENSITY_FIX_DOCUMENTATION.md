# VoxelSLAM Intensity 시각화 문제 해결 작업 요약

## 문제 설명
RViz에서 VoxelSLAM의 모든 포인트 클라우드 토픽(`/map_cmap`, `/map_scan`, `/map_init`, `/map_pmap` 등)이 intensity 정보 대신 단일 빨간색으로만 표시되는 문제가 발생

## 근본 원인
VoxelSLAM 파이프라인에서 센서 데이터가 처리되는 과정에서 intensity 정보가 여러 단계에서 누락되거나 고정값으로 대체되는 현상

## 주요 발견사항

### 1. Hesai LiDAR Intensity 범위
- Hesai 센서의 실제 intensity 범위: 0-170
- RViz 기본 설정 Max Intensity: 255
- 이로 인해 모든 포인트가 고강도(빨간색)로 표시됨

### 2. 데이터 플로우에서 intensity 누락 지점들
센서 → PointType → pointVar → 맵 토픽 → RViz 과정에서 여러 단계에서 intensity 정보 손실

## 수행한 수정 작업

### 1. pointVar 구조체에 intensity 필드 추가
**파일**: `/home/mt-dhpyo-ubuntu/mappingtool_v2_ws/src/Voxel-SLAM/VoxelSLAM/src/voxelslam.hpp`
```cpp
// 라인 85 근처
struct pointVar {
    // 기존 필드들...
    float intensity = 0.0;  // 추가됨
};
```

### 2. var_init 함수에서 intensity 복사 추가
**파일**: `/home/mt-dhpyo-ubuntu/mappingtool_v2_ws/src/Voxel-SLAM/VoxelSLAM/src/voxelslam.hpp`
```cpp
// 라인 95 근처 var_init 함수
void var_init(pointVar &pv, const PointType &ap) {
    pv.x = ap.x; pv.y = ap.y; pv.z = ap.z;
    pv.intensity = ap.intensity;  // 추가됨
    // 나머지 필드들...
}
```

### 3. 키프레임 생성 시 intensity 복사
**파일**: `/home/mt-dhpyo-ubuntu/mappingtool_v2_ws/src/Voxel-SLAM/VoxelSLAM/src/voxelslam.cpp`
```cpp
// 라인 429 근처
PointType pt;
pt.x = pp.x; pt.y = pp.y; pt.z = pp.z;
pt.intensity = pp.intensity;  // 수정됨 (이전: pt.intensity = 50;)
```

### 4. map_init 토픽 발행 시 intensity 복사
**파일**: `/home/mt-dhpyo-ubuntu/mappingtool_v2_ws/src/Voxel-SLAM/VoxelSLAM/src/voxelslam.cpp`
```cpp
// 라인 749 근처
PointType pt;
pt.x = pv.x; pt.y = pv.y; pt.z = pv.z;
pt.intensity = pv.intensity;  // 추가됨
```

### 5. pub_localmap 함수에서 intensity 올바른 복사
**파일**: `/home/mt-dhpyo-ubuntu/mappingtool_v2_ws/src/Voxel-SLAM/VoxelSLAM/src/voxelslam.cpp`
```cpp
// 라인 127 근처
ap.intensity = pv.intensity;  // 수정됨 (이전: ap.intensity = cur_session;)
```

### 6. PCD 저장 시 intensity 복사
**파일**: `/home/mt-dhpyo-ubuntu/mappingtool_v2_ws/src/Voxel-SLAM/VoxelSLAM/src/voxelslam.cpp`
```cpp
// 라인 241 근처
ap.intensity = pw.intensity;  // 추가됨
```

### 7. down_sampling_voxel 함수에서 intensity 보존
**파일**: `/home/mt-dhpyo-ubuntu/mappingtool_v2_ws/src/Voxel-SLAM/VoxelSLAM/src/tools.hpp`
```cpp
// 라인 238 근처
pp.intensity = (pp.intensity * pp.curvature + p_c.intensity) / (pp.curvature + 1);
// 수정됨 (이전: 고정값 할당)
```

## 알고리즘 무결성 보장

함수 오버로딩을 통해 기존 알고리즘 경로 보존:
```cpp
// 기존 호출 경로는 그대로 유지
void down_sampling_voxel(pcl::PointCloud<PointType> &pl_feat, double voxel_size);

// intensity 처리가 필요한 새로운 경로 추가  
void down_sampling_voxel(pcl::PointCloud<PointType> &pl_feat, double voxel_size, bool preserve_intensity);
```

## 검증 과정

### 1. 센서 데이터 범위 확인
- Hesai 센서 intensity 범위: 0-170 확인
- 디버깅 코드로 실제 값 모니터링 후 제거

### 2. RViz 설정 조정 권장사항
- Max Intensity를 255에서 170으로 변경 권장
- Hesai 센서 특성에 맞는 범위 설정

## 결과
- `/map_cmap` 토픽에서 다양한 색상의 intensity 기반 시각화 성공
- 모든 맵 관련 토픽에서 올바른 intensity 정보 전달
- 알고리즘 성능에 영향 없이 intensity 정보 보존

## 핵심 교훈
1. **데이터 플로우 추적의 중요성**: 센서에서 시각화까지 전체 경로에서 데이터 손실 지점 식별 필요
2. **센서별 특성 이해**: 각 센서의 intensity 범위와 특성 파악 중요
3. **단계별 수정**: 파이프라인의 각 단계에서 intensity 정보가 올바르게 전달되도록 보장
4. **알고리즘 무결성**: 기능 추가 시 기존 알고리즘 경로 보존 필요

## 관련 파일 목록
- `voxelslam.hpp`: pointVar 구조체 및 var_init 함수 수정
- `voxelslam.cpp`: 키프레임, 토픽 발행, PCD 저장 부분 수정
- `tools.hpp`: down_sampling_voxel 함수 intensity 보존 로직 추가
- `feature_point.hpp`: Hesai 핸들러에서 올바른 intensity 복사 확인
- `voxel_map.hpp`: down_sampling_pvec 함수에서 intensity 처리 이미 구현됨

## 추후 참고사항
- 새로운 센서 지원 시 intensity 범위 확인 필요
- RViz 설정에서 센서별 적절한 Max Intensity 값 설정
- 다른 센서 타입에서도 유사한 intensity 누락 이슈 확인 권장