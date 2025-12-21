import casadi as ca
import numpy as np

def create_mpc_solver():
    # ==========================================
    # 1. 設定パラメータ
    # ==========================================
    T = 0.1         # サンプリング時間 [s]
    N = 20          # 予測ホライゾン
    
    # 重み係数 (数式の w1 ~ w5)
    w1 = 10.0       # 距離誤差 rho^2
    w2 = 5.0        # 角度誤差 (phi - phi_target)^2
    w3 = 1.0        # 速度抑制 v^2
    w4 = 0.1        # 省エネ (並進入力 F^2)
    w5 = 0.1        # 省エネ (回転入力 M^2)

    # ロボットの物理パラメータ (F, M を動きに変換するため)
    mass = 10.0     # 質量 [kg]
    inertia = 1.0   # 慣性モーメント [kg*m^2]

    # ==========================================
    # 2. 変数の定義 (CasADi Optiスタック)
    # ==========================================
    opti = ca.Opti()

    # --- 状態変数 X ---
    # [x, y, phi, v, omega] (5次元)
    # v: 並進速度, omega: 角速度
    n_states = 5
    X = opti.variable(n_states, N+1)

    # --- 制御入力 U ---
    # [F, M] (2次元)
    # F: 並進駆動力 (数式の F_k)
    # M: 回転トルク (数式の M_k)
    n_controls = 2
    U = opti.variable(n_controls, N)

    # --- 外部パラメータ ---
    # 現在の状態 [x, y, phi, v, w]
    P_current = opti.parameter(n_states)
    
    # 目標軌道 (Reference) 
    # [x_target, y_target, phi_target] を N+1 ステップ分
    P_target = opti.parameter(3, N+1)

    # ==========================================
    # 3. コスト関数の構築 (数式 J の実装)
    # ==========================================
    J = 0
    
    for k in range(N):
        # 変数の取り出し
        x_k = X[0, k]
        y_k = X[1, k]
        phi_k = X[2, k]
        v_k = X[3, k]
        
        # 入力の取り出し
        F_k = U[0, k]
        M_k = U[1, k]

        # 目標値の取り出し
        x_target = P_target[0, k]
        y_target = P_target[1, k]
        phi_target = P_target[2, k]

        # --- 数式の各項を計算 ---
        
        # 1. 距離誤差 (rho^2)
        # rho^2 = (x - x_target)^2 + (y - y_target)^2
        rho_sq = (x_k - x_target)**2 + (y_k - y_target)**2
        
        # 2. 角度誤差 (phi - phi_target)^2
        # ※ 2piの不連続性を考慮する場合は ca.sin() などを使いますが、ここでは数式通り実装します
        ang_err_sq = (phi_k - phi_target)**2
        
        # 3. 速度抑制 v^2
        vel_sq = v_k**2
        
        # 4 & 5. 省エネ (F^2 + M^2)
        input_energy = w4 * (F_k**2) + w5 * (M_k**2)

        # --- 合計コスト J に加算 ---
        # J += w1*rho^2 + w2*angle_err + w3*v^2 + (w4*F^2 + w5*M^2)
        J += w1 * rho_sq + \
             w2 * ang_err_sq + \
             w3 * vel_sq + \
             input_energy

    # 最小化対象として設定
    opti.minimize(J)

    # ==========================================
    # 4. モデル制約 (ダイナミクス)
    # ==========================================
    # 入力 F, M が状態 v, omega をどう変化させるかを定義
    for k in range(N):
        x_next   = X[0, k+1]
        y_next   = X[1, k+1]
        phi_next = X[2, k+1]
        v_next   = X[3, k+1]
        w_next   = X[4, k+1]

        # 現在の値
        v_k = X[3, k]
        phi_k = X[2, k]
        w_k = X[4, k]
        F_k = U[0, k]
        M_k = U[1, k]

        # 運動方程式 (オイラー法)
        # 位置の更新
        opti.subject_to(x_next   == X[0, k] + v_k * ca.cos(phi_k) * T)
        opti.subject_to(y_next   == X[1, k] + v_k * ca.sin(phi_k) * T)
        opti.subject_to(phi_next == X[2, k] + w_k * T)
        
        # 速度の更新 (F = ma より a = F/m)
        opti.subject_to(v_next   == v_k + (F_k / mass) * T)
        
        # 角速度の更新 (M = I*alpha より alpha = M/I)
        opti.subject_to(w_next   == w_k + (M_k / inertia) * T)

    # 初期状態制約
    opti.subject_to(X[:, 0] == P_current)

    # ソルバー設定 (Ipopt)
    opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
    opti.solver('ipopt', opts)

    return opti, P_current, P_target, U, X

# ==========================================
# 実行テスト (ダミーデータ)
# ==========================================
if __name__ == "__main__":
    opti, P_cur, P_ref, U_sol, X_sol = create_mpc_solver()

    # 1. 現在の状態 [x=0, y=0, phi=0, v=0, w=0]
    opti.set_value(P_cur, [0, 0, 0, 0, 0])

    # 2. 目標軌道 (20ステップ先まで x方向に1mずつ進む例)
    N_horizon = 20
    target_data = np.zeros((3, N_horizon + 1))
    for i in range(N_horizon + 1):
        target_data[0, i] = i * 0.1  # x_target
        target_data[1, i] = 0.0      # y_target
        target_data[2, i] = 0.0      # phi_target
    
    opti.set_value(P_ref, target_data)

    # 3. 計算実行
    try:
        sol = opti.solve()
        print("計算成功！")
        print("最初の入力 F (Force):", sol.value(U_sol[0, 0]))
        print("最初の入力 M (Torque):", sol.value(U_sol[1, 0]))
    except Exception as e:
        print("計算失敗:", e)