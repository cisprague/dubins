# {student full name}
# {student id}
# {student email}

from dubins import Car

''' <<< write your code below >>> '''


''' <<< write your code below >>> '''

def solution():
    car = Car()

    ''' <<< write your code below >>> '''


    ''' <<< write your code below >>> '''

    return car, controls, times

if __name__ == "__main__":

    # evaluate your code
    car, controls, times = solution()
    xl, yl, thetal, ul, tl, done = car.evaluate(controls, times)
    print("Level 1: ", "succesful" if done[0] else "unsuccesful")
    print("Level 2: ", "succesful" if done[1] else "unsuccesful")
    print("Level 3: ", "succesful" if done[2] else "unsuccesful")
