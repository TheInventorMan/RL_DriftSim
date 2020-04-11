from CarModel import Car
from TireModel import Tire
from WheelModel import *
import matplotlib.pyplot as plt

def test4():
    tire = Tire()
    x = []
    data = []
    for i in range(-100, 100):
        f = tire.getContactForces(500, 0.65, i/10000, 0, 0)
        print(i/10000)
        data.append(f[0])
        x.append(i/10000)
    print(data)
    plt.plot(x, data)
    plt.show()

def test3(): #wheel, 5 m/s +x direction
    whl = RearWheel(2,-1)
    whl.normal = 250 #N
    whl.ang_vel = 0 #15.444 #rad/s

    x, y = whl.computeForces(1, 0, 1, 0)

    print(x, y)

def test2():
    myCar = Car()
    myCar.applyControl(0, 200, 1)

    for i in range(5):
        myCar.applyControl(0,0,0.5)

    print(myCar.fx_cg, myCar.fy_cg, myCar.mz_cg)

def test1():
    myCar = Car()
    myCar.explicitControl(0, 200, 0.01)
    for i in range(201):
        myCar.applyControl(0, 0, 0.01)
        if i%100 == 0:
            #_ = input("_")
            print(myCar.getState()["control"])
            print(myCar.getState()["world"])
            print(myCar.getState()["kin"])
            print(myCar.getState()["dyn"])
            print(" ")
    _ = input("_")
    myCar.explicitControl(0, 0, 0.01)
    print("*******************************************************************************************************************")
    for i in range(21):
        myCar.applyControl(0, 0, 0.01)
        if i%100 == 0:
            #_ = input("_")
            print(myCar.getState()["control"])
            print(myCar.getState()["world"])
            print(myCar.getState()["kin"])
            print(myCar.getState()["dyn"])
            print(" ")


test1()
#test2()
#test3()
#test4()
