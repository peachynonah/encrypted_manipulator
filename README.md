# 내용
1. 플랜트 <enc_turtle_plant>: 터틀봇의 위치 (x,y) 를 암호화 후 송신
2. 컨트롤러 <enc_turtle_controller>: 암호문 데이터 **x**, **y**의 덧셈, 곱셈 1회 수행 후 송신
3. 이때 암호문은 cereal을 이용하여 string으로 변환 후 uint8[]으로 통신 (ROS에서 char 통신은 안되는 것으로 보임)


