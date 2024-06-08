# ContraintBasedMotionEdit

Retargetting Motion to New Characters[Gleicher et al] 참고

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/demo.gif" width="500" height="400">

다음은 구현 방법에 대한 간단할 설명입니다.

먼저 inverse kinematics를 사용해 모션 데이터의 특정 프레임에서 관절을 이동시킵니다. 수정한 프레임들의 관절 정보와 기존 모션 데이터의 관절 정보의 차이를 계산하여 displacement map에 저장합니다. 이 때 수정된 프레임의 모션 데이터가 constraint의 개수가 됩니다. 이렇게 특정 프레임의 모션 데이터를 수정하여 constraint를 추가하고, 특정한 프레임 간격으로 노트를 설정한 후 displacement map에 저장된 값을 지나도록 하는 cubic b-spline을 생성합니다.

이 때 기존의 b-spline은 노트들의 값을 지정하고 해당 노트들로 곡선을 생성하지만 이러한 문제의 경우 displacement map에 저장된 값들을 최소한의 차이로 지나게 하는 least square 문제를 풀어야 합니다. displacement map의 값을 p(t),  basis fucntion의 값을 B(t), 노트의 값을 b라고 한다면  p(t) = b * B(t)로 계산될 수 있습니다. B(t)는 대부분의 경우에서 singular하고 정방행렬이 아니기 때문에 역행렬을 구하기 위해 SVD를 한 후 least square를 풀어 역행렬을 구하고 양변에 곱해주어 constaint를 만족하는 노트 b를 구할 수 있습니다.

다음은 관절들에 대해 쿼터니언으로 표현된 displacement를 계산하는 방법입니다.

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/image1.png">

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/image2.png">

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/image3.png">

하지만 이렇게 구한 b는 least square를 통해 구한 노트를 사용하였기 때문에 정확하게 수정한 지점을 지나지 않으며 노트의 간격이 넓으면 low frequancy 대역만 변화합니다. 따라서 multi-level approximation을 적용하여 기존의 노트 간격을 줄이고 이전에 생성된 b-spline과 displacement map의 차이를 다시 계산하여 이전과 같은 방법으로 새로운 b-spline을 구하고 이전의 b-spline에 더해줍니다.

<img src="https://github.com/jong1-choi/ContraintBasedMotionEdit/blob/main/image4.png">

특정 횟수만큼을 반복하여 low frequancy와 high frequancy 모두 적용된 b-spline을 원래 모션에 더해 constaint를 만족하여 자연스럽게 수정된 모션 데이터를 생성할 수 있습니다.
