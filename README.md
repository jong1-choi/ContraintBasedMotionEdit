# ContraintBasedMotionEdit

Retargetting Motion to New Characters[Gleicher et al] 참고

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/Images/demo.gif" width="500" height="400">

### 구현 방법

먼저 inverse kinematics를 사용해 모션 데이터의 특정 프레임에서 관절을 이동시킵니다. 수정한 프레임들의 관절 정보와 기존 모션 데이터의 관절 정보의 차이를 계산하여 displacement map에 저장합니다. 이 때 수정된 프레임의 모션 데이터가 constraint의 개수가 됩니다. 이렇게 특정 프레임의 모션 데이터를 수정하여 constraint를 추가하고, 특정한 프레임 간격으로 노트를 설정한 후 displacement map에 저장된 값을 지나도록 하는 cubic b-spline을 생성합니다.

이 때 기존의 b-spline은 노트들의 값을 지정하고 해당 노트들로 곡선을 생성하지만 이러한 문제의 경우 displacement map에 저장된 값들을 최소한의 차이로 지나게 하는 least square 문제를 풀어야 합니다. displacement map의 값을 p(t),  basis fucntion의 값을 B(t), 노트의 값을 b라고 한다면  p(t) = b * B(t)로 계산될 수 있습니다. B(t)는 대부분의 경우에서 singular하고 정방행렬이 아니기 때문에 역행렬을 구하기 위해 SVD를 한 후 least square를 풀어 역행렬을 구하고 양변에 곱해주어 constaint를 만족하는 노트 b를 구할 수 있습니다.

다음은 관절들에 대해 쿼터니언으로 표현된 displacement를 계산하는 방법입니다.

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/Images/image1.png">

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/Images/image2.png">

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/Images/image3.png">

하지만 이렇게 구한 b는 least square를 통해 구한 노트를 사용하였기 때문에 정확하게 수정한 지점을 지나지 않으며 노트의 간격이 넓으면 low frequancy 대역만 변화합니다. 따라서 multi-level approximation을 적용하여 기존의 노트 간격을 줄이고 이전에 생성된 b-spline과 displacement map의 차이를 다시 계산하여 이전과 같은 방법으로 새로운 b-spline을 구하고 이전의 b-spline에 더해줍니다.

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/Images/image4.png">

특정 횟수만큼을 반복하여 low frequancy와 high frequancy 모두 적용된 b-spline을 원래 모션에 더해 constaint를 만족하여 자연스럽게 수정된 모션 데이터를 생성할 수 있습니다.

### 사용된 Inverse Kinematics Solver

먼저 3차원에 여러개의 관절들이 있을 때 end effector의 움직인은 비선형적으로 나타납니다. 하지만 자코비안을 행렬을 사용하면 각 관절이 회전하는 각도 세타에 대해 end effector의 좌표값이 국소적으로 이동하는 값을 계산할 수 있고 이는 비선형 변환을 선형 변환으로 근사시키고 반복적으로 이동시켜 문제를 해결할 수 있습니다.

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/Images/image5.png">

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/Images/image6.png">

이 때 자코비안 행렬은 3차원에서 end effector의 좌표에 대해 회전값 세타에 대한 편미분값으로 표현됩니다. 자코비안 값은 end effector의 부모 관절들을 찾은 다음 루트 관절에서부터 end effector까지의 차이를 벡터로 구하고 x,y,z축에 외적을 하여 구할 수 있습니다.

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/Images/image7.png">

이렇게 계산된 자코비안 행렬의 역행렬을 end effector가 이동한 벡터에 곱해서 각 관절들이 회전해야 하는 세타를 구할 수 있는데 이 때 자코비안 행렬은 정방행렬이 아니고 singular할 수 있기 때문에 SVD를 통해 분해하고 least square 문제를 풀어 각 관절의 회전이 최소가 되는 관절들의 세타를 얻습니다.

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/Images/image8.png">

앞서 설명한것과 같이 자코비안 행렬이 국소적으로 선형 변환이 되는 성질을 이용했기 때문에 잘게 나누어 반복할 횟수를 정하고 방금 구한 세타값을 반복 횟수로 나눈 만큼 각 관절들을 회전 시켜줍니다. 이러한 과정을 정해진 횟수만큼 반복하게 되면 Inverse kinematics 문제를 해결 할 수 있습니다.

