import math
import argparse

def step_model(x, y, phi, v, w, F, M, dt, m, I, Cv, Cw):
    # State Prediction Equations from the image
    # x_{k+1} = x_k + (v_k * cos(phi_k)) * dt
    x_next = x + (v * math.cos(phi)) * dt
    
    # y_{k+1} = y_k + (v_k * sin(phi_k)) * dt
    y_next = y + (v * math.sin(phi)) * dt
    
    # phi_{k+1} = phi_k + w_k * dt
    phi_next = phi + w * dt
    
    # v_{k+1} = v_k + ((F_k - Cv * v_k) / m) * dt
    v_next = v + ((F - Cv * v) / m) * dt
    
    # w_{k+1} = w_k + ((M_k - Cw * w_k) / I) * dt
    w_next = w + ((M - Cw * w) / I) * dt
    
    return x_next, y_next, phi_next, v_next, w_next
+
def main():
    parser = argparse.ArgumentParser(description='Verify Dynamic Bicycle Model Equations')
    
    # Initial States
    parser.add_argument('--x', type=float, default=0.0, help='Initial x position')
    parser.add_argument('--y', type=float, default=0.0, help='Initial y position')
    parser.add_argument('--phi', type=float, default=0.0, help='Initial yaw angle (rad)')
    parser.add_argument('--v', type=float, required=True, help='Initial velocity (v_k)')
    parser.add_argument('--w', type=float, required=True, help='Initial angular velocity (omega_k)')
    
    # Inputs (Forces/Moments)
    parser.add_argument('--F', type=float, default=0.0, help='Force input (F_k)')
    parser.add_argument('--M', type=float, default=0.0, help='Moment input (M_k)')
    
    # Parameters
    parser.add_argument('--dt', type=float, default=0.1, help='Time step (Delta t)')
    parser.add_argument('--m', type=float, default=1.0, help='Mass (m)')
    parser.add_argument('--I', type=float, default=1.0, help='Inertia (I)')
    parser.add_argument('--Cv', type=float, default=0.1, help='Velocity dragging coefficient (Cv)')
    parser.add_argument('--Cw', type=float, default=0.1, help='Angular velocity dragging coefficient (Cw)')
    
    args = parser.parse_args()
    
    print("=== Initial State ===")
    print(f"x:   {args.x:.4f}")
    print(f"y:   {args.y:.4f}")
    print(f"phi: {args.phi:.4f}")
    print(f"v:   {args.v:.4f}")
    print(f"w:   {args.w:.4f}")
    print("\n=== Inputs ===")
    print(f"F:   {args.F:.4f}")
    print(f"M:   {args.M:.4f}")
    print("\n=== Parameters ===")
    print(f"dt:  {args.dt:.4f}")
    print(f"m:   {args.m:.4f}")
    print(f"I:   {args.I:.4f}")
    print(f"Cv:  {args.Cv:.4f}")
    print(f"Cw:  {args.Cw:.4f}")
    
    # Calculation
    x_next, y_next, phi_next, v_next, w_next = step_model(
        args.x, args.y, args.phi, args.v, args.w, 
        args.F, args.M, args.dt, 
        args.m, args.I, args.Cv, args.Cw
    )
    
    print("\n=== Next State (k+1) ===")
    print(f"x:   {x_next:.4f}")
    print(f"y:   {y_next:.4f}")
    print(f"phi: {phi_next:.4f}")
    print(f"v:   {v_next:.4f}")
    print(f"w:   {w_next:.4f}")

if __name__ == "__main__":
    main()
