class Queue:
    def __init__(self, capacity):
        self.front = 0
        self.rear = -1
        self.capacity = capacity
        self.queue = [None] * capacity

    # Function to insert an element at the rear of the queue
    def enqueue(self, data):
        # Check if the queue is full
        if self.rear == self.capacity - 1:
            print("Queue is full")
            return
        
        # Insert element at the rear
        self.rear += 1
        self.queue[self.rear] = data

    # Function to delete an element from the front of the queue
    def dequeue(self):
        # If the queue is empty
        if self.front > self.rear:
            print("Queue is empty")
            return
        
        # Shift all elements from index 1 till rear to the left by one
        for i in range(self.rear):
            self.queue[i] = self.queue[i + 1]

        # Decrement rear
        self.rear -= 1

    # Function to print queue elements
    def display(self):
        if self.front > self.rear:
            print("Queue is Empty")
            return

        # Traverse front to rear and print elements
        for i in range(self.front, self.rear + 1):
            print(self.queue[i], end=" <-- ")
        # print()

    # Function to print the front of the queue
    def front_element(self):
        if self.rear == -1:
            print("Queue is Empty")
            return
        print("Front Element is:", self.queue[self.front])

    def append(self, data):
        if self.rear == self.capacity - 1:
            # Shift all elements from index 1 till rear to the left by one
            for i in range(self.rear):
                self.queue[i] = self.queue[i + 1]   
            # Decrement rear
            self.rear -= 1

        # Insert element at the rear
        self.rear += 1
        self.queue[self.rear] = data
