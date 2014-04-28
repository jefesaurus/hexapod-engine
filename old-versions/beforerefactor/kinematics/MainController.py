import thread
from math import pi,cos,sin,sqrt
from animateChassis import SuperModel
from getch import _Getch

from multiprocessing import Process, Queue 


def engine(input_queue,output_queue):
  print "starting engine.."
  speed = .05
  angular_velocity_coeff = .03

  while True:
    control_signal = input_queue.get()
    if control_signal == 'q': return

    control_vector = ((0,0),0)
    angle = 0
    turn_direction = 0

    if control_signal == 'w': angle = 0
    elif control_signal == 'd': angle = pi/2
    elif control_signal == 's': angle = pi
    elif control_signal == 'a': angle = 3*pi/2
    elif control_signal == 'o': turn_direction = 1
    elif control_signal == 'p': turn_direction = -1

    control_vector = ((speed*cos(angle), speed*sin(angle)),angular_velocity_coeff*turn_direction)
    print str(control_vector) + '\r'
    output_queue.put(control_vector)

def controller(control_queue):
  getch = _Getch()
  while True:
    print 'Press WASD or q'
    next_char = _Getch()
    while True:
      k = next_char()
      if k == 'q':
        control_queue.put(k)
        return
      elif k in 'wasdop': control_queue.put(k)

def orchestrate():

  input_queue = Queue()
  output_queue = Queue()
  model = SuperModel(output_queue)
  p1 = Process(target=engine, args=(input_queue,output_queue))
  p1.start()
  thread.start_new_thread(controller, (input_queue,))
  model.show()
  p1.join() # block until engine is finished.
  
if __name__ == '__main__':
  orchestrate()
