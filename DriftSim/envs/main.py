from CarModel import Car

myCar = Car()
myCar.applyControl(0, 200, 0.01)
for i in range(1000):
    myCar.applyControl(0, 0, 0.01)
    if i%100 == 0:
        #_ = input("_")
        print(myCar.getState()["control"])
        print(myCar.getState()["world"])
        print(myCar.getState()["cg"])
        print(" ")
