# main.py

import argparse
from lab_tasks import task2, task3, task4, task5, task6

def main():
    '''Use parser to select and run a lab task'''
    parser = argparse.ArgumentParser()
    parser.add_argument('task', choices=['task2', 'task3', 'task4', 'task5', 'task6'])
    args = parser.parse_args()

    if args.task == 'task2':
        print("Running Task 2")
        task2.run()
    elif args.task == 'task3':
        print("Running Task 3")
        task3.run()
    elif args.task == 'task4':
        print("Running Task 4")
        task4.run()
    elif args.task == 'task5':
        print("Running Task 5")
        task5.run()
    elif args.task == 'task6':
        print("Running Task 6")
        task6.run()

if __name__ == '__main__':
    main()
