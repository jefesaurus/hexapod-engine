import thread
from getch import _Getch

from multiprocessing import Process, Queue 


def engine(control_queue):
  print "starting engine"
  while True:
    control_vector = control_queue.get()
    print control_vector + '\r'
    if control_vector == 'q':
      return

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
      elif k in 'wasd': control_queue.put(k)

if __name__ == '__main__':
  control_queue = Queue()
  p1 = Process(target=engine, args=(control_queue,))
  #p2 = Process(target=controller, args=(control_queue,))
  p1.start()
  thread.start_new_thread(controller, (control_queue,))
  p1.join() # block until engine is finished.
