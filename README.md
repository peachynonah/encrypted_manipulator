Original Source: https://github.com/lsw23101/Encrypted_turtle.git 

Please check README.md.back, or attached github link for former information of encryption code.

# Encrypted manipulator 개발 계획

## Original code overview
1. (System node) turtle sim의 position x, y를 각각 encrypt (scalar)
2. Enc(x), Enc(y)를 controller로 전송
3. (Controller node) encrypted scalar 간의 addition, multiplication Enc(x) + Enc(y) , Enc(x) * Enc(y)를 수행
4. 결과값을 system node로 전송
5. system node에서 decrypt


## Encrypted manipulator with P controller overview

개발 순서: P controller $\to$ PD controller $\to$ PD+G controller

Original code를 확장해서, two link manipulator에 P controller부터 적용해보고자 함.


1. (System node) **two link manipulator**의 joint position **vector** (q1,q2), P gain **vector** (k1,k2)를 각각 encrypt
2. Enc([q1 q2]), Enc([k1 k2])를 controller로 전송
3. (Controller node) **encrypted vector간의 multiplication** Enc([k1 k2]) * Enc(q1 q2) (element-wise)를 수행
4. 결과값을 system node로 전송
5. system node에서 decrypt
Develop stage: 1. P controller 2. PD controller 3. PD+G controller


## P controller development plan
### A. Implement encrypted multiplication of vector
Original code $\to$ program to develop
1. Scalar encryption $\to$ vector encryption 
2. (system to controller) Encrypted scalar 값 전송 $\to$ encrypted vector 값 전송 
3. Encrypted scalar multiplication $\to$ Encrypted vector multiplication
4. (controller to system) Encrypted scalar 값 전송 $\to$ encrypted vector 값 전송
5. system node에서 decrypt하는 부분이 위 과정과 호환되도록 수정

### B. Replace turtlesim with two link manipulator
(Rough guideline) 

- turtlesim model 관련 부분 함수, 변수 파악
- two link manipulator input / output data type 확인 및 암호화 가능하도록 변경
- model 정보를 모듈화해서 encryption code와 파일을 분리하는 것이 코드 가독성과 개발 편의를 위해 필요


### Current status
**완료**

A-1. encrypt ee_pos = (x,y) position of turtlebot, and p_gain = (constant, constant) and multiply encrypted data. 
A-2. 위 데이터를 전송할 수 있도록 publisher / subscriber data type을 변경해야 함.

**Work in Progress**

A-3. Encrypted scalar multiplication $\to$ Encrypted vector * vector multiplication

**논의사항**
- Plaintext * EncryptedData 가 OpenFHE의 BGV Scheme에서 가능한지?
- 가능하다면, Plaintext * EncryptedData $\neq$ 아무 정수 * EncryptedData인지?
  - 즉 만약 공격자가 controller를 탈취 후 EncryptedData만 받아와서 아무 정수를 곱한 입력을 출력한다고 하면, 가능한 시나리오인지?  

