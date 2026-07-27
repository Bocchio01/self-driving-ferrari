The MPC controller node uses the [OSQP](https://osqp.org/) solver to solve the quadratic programming problem.
The OSQP solver is a C library for solving convex quadratic programs.
The OSQP-Eigen is a wrapper around the OSQP solver that allows it to be used with Eigen matrices.

In order to build the MPC controller node, you need to install the OSQP and OSQP-Eigen libraries.

## OSPQ

```bash
git clone --recursive https://github.com/osqp/osqp
cd osqp

mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

# OSPQ-Eigen

```bash
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

To make sure the system can find the libraries, run:

```bash
sudo ldconfig
```