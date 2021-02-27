import numpy as np
import random as random
import math
import copy
from tkinter import *
from PIL import ImageTk, Image
import time
import threading


class parent():
    def __init__(self, paren, puzzles):
        self.paren = paren
        self.puzzlee = puzzles


init_puzzle = np.array([[0, 1, 2],
                        [3, 4, 5],
                        [6, 7, 8]])

global glob_cost
glob_cost = 1


def shuffle(puzzlee, cost):
    lmove = np.array([0, 0])
    puzzle = copy.deepcopy(puzzlee)
    iteration = 10 # random.randint(20, 30)
    i = 0
    movelist = []
    currentpos = np.array([0, 0])
    chosenmove = np.array([0, 0])
    while (i < iteration):
        if currentpos[0] - 1 >= 0:
            movelist.append([currentpos[0] - 1, currentpos[1]])
        if currentpos[0] + 1 <= 2:
            movelist.append([currentpos[0] + 1, currentpos[1]])
        if currentpos[1] - 1 >= 0:
            movelist.append([currentpos[0], currentpos[1] - 1])
        if (currentpos[1] + 1 <= 2):
            movelist.append([currentpos[0], currentpos[1] + 1])
        while chosenmove[0] == lmove[0] and chosenmove[1] == lmove[1]:
            chosenmove = random.choice(movelist)
        puzzle[currentpos[0], currentpos[1]] = copy.deepcopy(puzzle[chosenmove[0], chosenmove[1]])
        puzzle[chosenmove[0], chosenmove[1]] = 0
        lmove = copy.deepcopy(currentpos)
        currentpos = copy.deepcopy(chosenmove)
        chosenmove = copy.deepcopy(lmove)
        movelist = []
        i += 1

    print(i)
    cost = i
    return puzzle, cost


def Goal_Test(puzzle):
    Goal_puzzle = np.array([[0, 1, 2],
                            [3, 4, 5],
                            [6, 7, 8]])
    for i in range(3):
        for j in range(3):
            if Goal_puzzle[i][j] != puzzle[i][j]:
                return False
    return True


def Move(puzzle, coordinates):
    position = copy.deepcopy(puzzle)
    blankspace = []
    for i in range(3):
        for j in range(3):
            if position[i][j] == 0:
                blankspace = [i, j]
                break
    position[blankspace[0], blankspace[1]] = copy.deepcopy(position[coordinates[0], coordinates[1]])
    position[coordinates[0], coordinates[1]] = 0

    return position


def LegalMoves(puzzle):
    frontier = []
    blankpos = []
    for i in range(3):
        for j in range(3):
            if puzzle[i][j] == 0:
                blankpos = [i, j]
                break

    if (blankpos[0] - 1 >= 0):
        frontier.append([blankpos[0] - 1, blankpos[1]])
    if blankpos[0] + 1 <= 2:
        frontier.append([blankpos[0] + 1, blankpos[1]])
    if (blankpos[1] - 1 >= 0):
        frontier.append([blankpos[0], blankpos[1] - 1])
    if (blankpos[1] + 1 <= 2):
        frontier.append([blankpos[0], blankpos[1] + 1])
    return frontier


parents = []


def draw_the_path(steps):
    path = []
    path2 = []
    puzzless = np.array([[0, 1, 2],
                         [3, 4, 5],
                         [6, 7, 8]])
    current_index = 0
    for i in range(len(parents)):
        statement = True
        for j in range(3):
            for k in range(3):
                if parents[i].puzzlee[j][k] != puzzless[j][k]:
                    statement = False
        if statement:
            current_index = copy.deepcopy(i)
    path.append(parents[current_index].puzzlee)
    while steps > 0:
        current_index = parents[current_index].paren
        path.append(parents[current_index].puzzlee)
        steps -= 1
    return path


def Manhattan_distance(puzzle):
    result = 0
    count = 1
    for i in range(3):
        for j in range(3):
            index = puzzle[i][j] - 1
            distance = (2 - i) + (2 - j) if index == -1 else abs(i - (index / puzzle.size)) + abs(
                j - (index % puzzle.size))
            result += distance
            count += 1
    return result


class nodes():
    distance: int

    def __init__(self, puzzle, cost):
        self.position = puzzle
        self.cost = cost
        self.distance = Manhattan_distance(self.position)
        self.f_cost = cost + self.distance

    def __eq__(self, other):
        if (
                self.position == other.position).all() and self.cost == other.cost and self.distance == other.distance and self.f_cost == other.f_cost:
            return True
        else:
            return False


class min():
    def __init__(self):
        self.min = math.inf
        self.position = 0


def A_star(puzzle):
    open = list()
    closed = list()
    parents.append(parent(0, puzzle))
    current_node = nodes(puzzle, 0)
    open.append(current_node)
    successors = list()
    print(len(open))
    while len(open) != 0:
        #open.sort(key=lambda x: x.f_cost, reverse=False)
        min_int = math.inf
        min_index=0
        for i in range(len(open)):
            if open[i].f_cost < min_int:
                min_index = i
                min_int = open[i].f_cost
            if Goal_Test(open[i].position):
                return open[i].cost

        current_node = open[0]
        for i in range(len(parents)):
            statement = True
            if not np.array_equal(parents[i].puzzlee, current_node.position):
                statement = False
            if statement:
                current_index = copy.deepcopy(i)
        if Goal_Test(current_node.position):
            return current_node.cost
        succeseor_puzzles = LegalMoves(current_node.position)
        for i in succeseor_puzzles[:]:
            succ_puzzle = copy.deepcopy(Move(current_node.position, i))
            successors.append(nodes(succ_puzzle, current_node.cost + 1))

        for succ in successors[:]:
            state = True
            for i in range(len(open)):
                if np.array_equal(open[i].position, succ.position):
                    state = False
            for i in range(len(closed)):
                if np.array_equal(closed[i].position, succ.position):
                    state = False
            if state:
                open.append(succ)
                parents.append(parent(current_index, succ.position))

        del open[0]
        closed.append(current_node)
        successors.clear()


##print(old)
##print(Manhattan_distance(init_puzzle))
def gui():
    global root
    root = Tk()
    my_img0 = ImageTk.PhotoImage(Image.open('C:/Users/LENOVO/Desktop/I0.png'))
    my_img1 = ImageTk.PhotoImage(Image.open(('C:/Users/LENOVO/Desktop/I1.png')))
    my_img2 = ImageTk.PhotoImage(Image.open(('C:/Users/LENOVO/Desktop/I2.png')))
    my_img3 = ImageTk.PhotoImage(Image.open(('C:/Users/LENOVO/Desktop/I3.png')))
    my_img4 = ImageTk.PhotoImage(Image.open(('C:/Users/LENOVO/Desktop/I4.png')))
    my_img5 = ImageTk.PhotoImage(Image.open(('C:/Users/LENOVO/Desktop/I5.png')))
    my_img6 = ImageTk.PhotoImage(Image.open(('C:/Users/LENOVO/Desktop/I6.png')))
    my_img7 = ImageTk.PhotoImage(Image.open(('C:/Users/LENOVO/Desktop/I7.png')))
    my_img8 = ImageTk.PhotoImage(Image.open(('C:/Users/LENOVO/Desktop/I8.png')))
    global imgs
    imgs = [my_img0, my_img1, my_img2, my_img3, my_img4, my_img5, my_img6, my_img7, my_img8]

    mylabel0 = Label(root, image=imgs[solution[len(solution) - 1][0, 0]])
    mylabel0.grid(row=1, column=0)
    mylabel1 = Label(root, image=imgs[solution[len(solution) - 1][0, 1]])
    mylabel1.grid(row=1, column=1)
    mylabel2 = Label(root, image=imgs[solution[len(solution) - 1][0, 2]])
    mylabel2.grid(row=1, column=2)
    mylabel3 = Label(root, image=imgs[solution[len(solution) - 1][1, 0]])
    mylabel3.grid(row=2, column=0)
    mylabel4 = Label(root, image=imgs[solution[len(solution) - 1][1, 1]])
    mylabel4.grid(row=2, column=1)
    mylabel5 = Label(root, image=imgs[solution[len(solution) - 1][1, 2]])
    mylabel5.grid(row=2, column=2)
    mylabel6 = Label(root, image=imgs[solution[len(solution) - 1][2, 0]])
    mylabel6.grid(row=3, column=0)
    mylabel7 = Label(root, image=imgs[solution[len(solution) - 1][2, 1]])
    mylabel7.grid(row=3, column=1)
    mylabel8 = Label(root, image=imgs[solution[len(solution) - 1][2, 2]])
    mylabel8.grid(row=3, column=2)

    global labels
    labels = [mylabel0, mylabel1, mylabel2, mylabel3, mylabel4, mylabel5, mylabel6, mylabel7, mylabel8]
    root.mainloop()
    root.quit()

    return


def secondary_func():
    count = len(solution) - 1
    print(count)
    print(len(solution))

    while count > -1:
        time.sleep(1)
        global labels
        labels[0].grid_forget()
        labels[1].grid_forget()
        labels[2].grid_forget()
        labels[3].grid_forget()
        labels[4].grid_forget()
        labels[5].grid_forget()
        labels[6].grid_forget()
        labels[7].grid_forget()
        labels[8].grid_forget()
        solution1 = copy.deepcopy(solution[count])
        labels[0] = Label(root, image=imgs[solution1[0][0]])
        labels[0].grid(row=1, column=0)
        labels[1] = Label(root, image=imgs[solution1[0][1]])
        labels[1].grid(row=1, column=1)
        labels[2] = Label(root, image=imgs[solution1[0][2]])
        labels[2].grid(row=1, column=2)
        labels[3] = Label(root, image=imgs[solution1[1][0]])
        labels[3].grid(row=2, column=0)
        labels[4] = Label(root, image=imgs[solution1[1][1]])
        labels[4].grid(row=2, column=1)
        labels[5] = Label(root, image=imgs[solution1[1][2]])
        labels[5].grid(row=2, column=2)
        labels[6] = Label(root, image=imgs[solution1[2][0]])
        labels[6].grid(row=3, column=0)
        labels[7] = Label(root, image=imgs[solution1[2][1]])
        labels[7].grid(row=3, column=1)
        labels[8] = Label(root, image=imgs[solution1[2][2]])
        labels[8].grid(row=3, column=2)
        root.update()
        count -= 1

    return


old = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8]])
init_puzzle, glob_cost = copy.deepcopy(shuffle(init_puzzle, glob_cost))
print(init_puzzle)
print("----------------------------")
cost = copy.deepcopy(A_star(init_puzzle))
print("The cost to achive the goal is \n{}".format(cost))
solution = draw_the_path(cost)
for i in solution[:]:
    print(i)
print("----------------------------")

x = threading.Thread(target=gui)
y = threading.Thread(target=secondary_func)
x.start()
y.start()
