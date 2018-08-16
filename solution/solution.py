# {student full name}
# {student id}
# {student email}

from dubins import Car

''' <<< write your code below >>> '''


''' <<< write your code below >>> '''

def solution(car):

    ''' <<< write your code below >>> '''


    ''' <<< write your code below >>> '''

    return car, controls, times

if __name__ == "__main__":

    # instantiate car object
    car = Car()

    # evaluate your code
    car, controls, times = solution(car)
    xl, yl, thetal, ul, tl, done = car.evaluate(controls, times, "E")
    print("Grade E: ", "succesful" if done else "unsuccesful")
    xl, yl, thetal, ul, tl, done = car.evaluate(controls, times, "C")
    print("Grade C: ", "succesful" if done else "unsuccesful")
    xl, yl, thetal, ul, tl, done = car.evaluate(controls, times, "A")
    print("Grade D: ", "succesful" if done else "unsuccesful")
