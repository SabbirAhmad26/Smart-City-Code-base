cvxopt install:
git clone https://github.com/DrTimothyAldenDavis/SuiteSparse.git
pushd SuiteSparse
git checkout v5.6.0
popd
export CVXOPT_SUITESPARSE_SRC_DIR=$(pwd)/SuiteSparse
pip3 install cvxopt


spline.py: polynomial trajectory calculation (input = array of (x,y))

lane.py:  print(x,y) locations and move the car to the clicked points (uncomment car.move())

test2.py: cars move with given angle and velocity

test.py: car moves with polynomial trajectory. Currently working on this one, a problem is the car spins around when it gets to the 1st point
