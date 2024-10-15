import math

if __name__ == "__main__":

    #Set the mission radius
    radius = 50 #meters

    #how many agents
    agents = 8

    #Angular space between agents
    angle_delta = 2*3.14 / float(agents)

    #find the locations

    for i in range(0,8):
        print(f"{round(radius*math.cos(i*angle_delta), 2)}, {round(radius*math.sin(i*angle_delta), 2)}")