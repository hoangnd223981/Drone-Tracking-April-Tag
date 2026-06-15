#include <BasicLinearAlgebra.h>

using namespace BLA;

BLA::Matrix<2,2> Fx;     BLA::Matrix<2,1> Gx;
BLA::Matrix<2,2> Px;     BLA::Matrix<2,2> Qx;
BLA::Matrix<2,1> Sx;     BLA::Matrix<1,2> Hx;
BLA::Matrix<2,2> Ix;     BLA::Matrix<1,1> Accx;
BLA::Matrix<2,1> Kx;     BLA::Matrix<1,1> Rx;
BLA::Matrix<1,1> Lx;     BLA::Matrix<1,1> Mx;


BLA::Matrix<2,2> Fy;     BLA::Matrix<2,1> Gy;
BLA::Matrix<2,2> Py;     BLA::Matrix<2,2> Qy;
BLA::Matrix<2,1> Sy;     BLA::Matrix<1,2> Hy;
BLA::Matrix<2,2> Iy;     BLA::Matrix<1,1> Accy;
BLA::Matrix<2,1> Ky;     BLA::Matrix<1,1> Ry;
BLA::Matrix<1,1> Ly;     BLA::Matrix<1,1> My;


BLA::Matrix<2,2> Fz;     BLA::Matrix<2,1> Gz;
BLA::Matrix<2,2> Pz;     BLA::Matrix<2,2> Qz;
BLA::Matrix<2,1> Sz;     BLA::Matrix<1,2> Hz;
BLA::Matrix<2,2> Iz;     BLA::Matrix<1,1> Accz;
BLA::Matrix<2,1> Kz;     BLA::Matrix<1,1> Rz;
BLA::Matrix<1,1> Lz;     BLA::Matrix<1,1> Mz;

void KalmanFilter(float X_vel, float Y_vel, float Z_pos, float AccXInertial, float AccYInertial, float AccZInertial)
{
  Accx = {AccXInertial * 981};
  Sx = Fx * Sx + Gx * Accx;
  Px = Fx * Px * ~Fx + Qx;
  Lx = Hx * Px * ~Hx + Rx;
  Kx = Px * ~Hx * Inverse(Lx);
  Mx = {X_vel * 100};
  Sx = Sx + Kx * (Mx - Hx * Sx);
  x_KF = Sx(0, 0);
  V_x_KF = Sx(1, 0);
  Px = (Ix - Kx * Hx) * Px;

  Accy = {AccYInertial * 981};
  Sy = Fy * Sy + Gy * Accy;
  Py = Fy * Py * ~Fy + Qy;
  Ly = Hy * Py * ~Hy + Ry;
  Ky = Py * ~Hy * Inverse(Ly);
  My = {Y_vel * 100};
  Sy = Sy + Ky * (My - Hy * Sy);
  y_KF = Sy(0, 0);
  V_y_KF = Sy(1, 0);
  Py = (Iy - Ky * Hy) * Py;

  Accz = {AccZInertial * 981};
  Sz = Fz * Sz + Gz * Accz;
  Pz = Fz * Pz * ~Fz + Qz;
  Lz = Hz * Pz * ~Hz + Rz;
  Kz = Pz * ~Hz * Inverse(Lz);
  Mz = {Z_pos * 100};
  Sz = Sz + Kz * (Mz - Hz * Sz);
  z_KF = Sz(0, 0);
  V_z_KF = Sz(1, 0);
  Pz = (Iz - Kz * Hz) * Pz;
}