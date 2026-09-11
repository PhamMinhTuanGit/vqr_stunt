# stand_4_wheel

Bộ điều khiển QP giữ thăng bằng cho VQRWheel (16 actuator).
Module này **KHÔNG** chứa bộ đứng dậy — repo SDK của bạn đã có `kStandUp`, dùng lại nguyên vẹn.

## Phạm vi

| Thành phần | File | Vai trò |
|---|---|---|
| `IdleHold` | `include/wq/idle_hold.hpp` | **Mốc 1**: đứng ổn định, bánh KHÔNG khóa, latch tư thế |
| `RobotModel` | `src/robot_model.cpp` | Pinocchio FK + contact Jacobian tại ĐIỂM CHẠM SÀN + g(q) |
| `BalanceQP` | `src/balance_qp.cpp` | QP 12 biến / 32 ràng buộc, phân bố lực tiếp đất |
| `StateQpBalance` | `src/fsm/state_qp_balance.cpp` | Adapter cắm vào FSM của SDK, blend nội bộ |
| `test_jacobian_fd` | `test/` | Kiểm chứng Jacobian bằng sai phân số |

## Tích hợp vào FSM sẵn có

Enum hiện tại của bạn:

    kInvalid = -1, kIdle = 0, kStandUp = 1, kJointDamping = 2, kLieDown = 4, kRLControl = 6

Giá trị **3 và 5 bị khuyết** — gần như chắc chắn đã được SDK dùng nội bộ (state ẩn
hoặc ID dành cho firmware/protocol). ĐỪNG lấp vào. Module này dùng:

    kQpBalance = 7

Kiểm chứng trước:  grep -rn "= 3\b\|= 5\b" <thư mục fsm>

### Sơ đồ chuyển trạng thái

    kIdle ──► kStandUp ──► kQpBalance ──► kLieDown
                              │  ▲
                      ngã/lỗi │  │ user
                              ▼  │
                        kJointDamping

Pha BLEND (1 s) nằm **bên trong** `kQpBalance::enter/run`, KHÔNG tạo state riêng.
Lý do: bảng chuyển trạng thái của SDK thường có validate; thêm state chỉ chạy 1 s
rồi tự thoát làm bảng phình vô ích.

### Ba câu hỏi phải trả lời trước khi nối dây

1. **Lúc `kStandUp` kết thúc, damping của 4 bánh bằng bao nhiêu?**
   VQR hiện dùng `kp=0`, `kd=wheel_lock_kd_` (mặc định 0.8), tức khóa nhớt chứ
   không khóa vị trí. `StateQpBalance` nhận giá trị qua `RobotIO::wheelKdNow()`
   và nội suy sang damping của QP; wheel `kp` luôn bằng 0.
2. **Chế độ cuối `kStandUp` là position PD thuần hay có torque?**
   QP ra torque chủ đạo + PD gain thấp. Chênh lệch này là thứ pha blend phải nuốt.
3. **`checkChange()` gọi trước hay sau `run()`?** Ảnh hưởng việc `enter()` nhìn
   thấy state cũ hay mới.

## Build

    sudo apt install robotpkg-py3*-pinocchio robotpkg-eiquadprog   # hoặc conda
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j

`-O3` bắt buộc: Eigen ở `-O0` chậm hơn ~20x, sẽ trượt deadline 500 Hz.
`WQ_NATIVE_ARCH` mặc định tắt vì `-march=native` có thể làm ABI căn hàng Eigen
khác với Pinocchio dựng sẵn và gây crash trong parser. Chỉ bật khi toàn bộ dependency
được build với cùng cờ ISA.
Nếu chưa có `eiquadprog`, CMake vẫn build `wq_robot_model` và test Jacobian;
phần `BalanceQP`/demo được bỏ qua kèm cảnh báo rõ ràng.

## Chạy kiểm chứng TRƯỚC khi cắm vào robot

    ./test_jacobian_fd ../../vqr_description/vqr_urdf/urdf/VQRWheel.urdf

Phải in `[PASS]` cho cả 4 chân. Đây là test rẻ nhất và bắt được gần hết lỗi
(sai trục bánh, sai bán kính, sai frame, sai quy ước LOCAL/WORLD).

## Ba quy ước Pinocchio đã xử lý sẵn (đừng sửa nếu chưa hiểu)

1. **`v` của floating base là hệ LOCAL.** Sim trả `v_WB` hệ world →
   `v.head<3>() = R.transpose() * v_WB`. Bỏ sót thì robot đứng yên vẫn ổn
   (v=0) nhưng vừa đẩy nhẹ là damping đẩy sai hướng. Rất khó debug.
2. **Dấu:** Pinocchio dùng `M dv + C v + g(q) = tau + sum J^T f`
   → quasi-static: `tau = g(q) - sum J^T f`. CÓ dấu trừ và CÓ g(q).
   Code kiểu MIT Cheetah viết `tau = -J^T f` không kèm g(q) vì Jacobian của họ
   là foot-relative-to-body. Trộn hai quy ước là lỗi kinh điển.
3. **Khớp bánh `continuous` có nq=2** (lưu cos/sin) → `nq=27, nv=22`, nq≠nv.
   Nhưng góc bánh KHÔNG ảnh hưởng contact Jacobian (tâm bánh nằm trên trục
   quay), nên ta giữ góc bánh ở neutral vĩnh viễn. Đó là lý do `RobotState`
   không cần `q_w`.

## Bảng gain khởi điểm

| Tham số | Giá trị | Chỉnh khi |
|---|---|---|
| `Kp_pos.z` | 400 | sụt → tăng; rung dọc → giảm |
| `Kd_pos.xy` | 15 | trôi → tăng; giật khi đẩy → giảm (hoặc v_WB nhiễu) |
| `Kp_rot.xy` | 500 | nghiêng khi đẩy vai → tăng |
| `beta` | 1e-2 | motor rít/chattering → TĂNG lên 5e-2 TRƯỚC khi động gain khác |
| `mu` | 0.4 | bánh trượt → giảm; robot yếu → tăng 0.6 |
| `kp_joint` | 5 | gain PHỤ, đừng vượt 15 — sẽ chống lại QP |

## Mốc tiếp theo: nhấc 2 chân chéo

KHÔNG viết controller mới. Chỉ:

    s.contact = {true, false, false, true};   // FL + HR

QP tự phân bố lực lên 2 điểm. Nhưng lúc đó bài toán **dư âm bậc**
(2 điểm × 3 lực = 6 biến, 6 phương trình wrench, cộng cone) → có thể vô nghiệm
và `lastSolveOk()` trả false liên tục. Khi đó **nới `S`**, không phải sửa gain.
`BalanceQpConfig::S_two_contact` đã chuẩn bị sẵn và tự động kích hoạt khi
số điểm tiếp xúc <= 2.
