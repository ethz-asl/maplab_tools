from multiprocessing import Lock

class Locker(object):
    instance = None
    mutex = Lock()

    def __new__(cls):
        if cls.instance is None:
            cls.instance = super(Locker, cls).__new__(cls)
            # Put any initialization here.
        return cls.instance

global_locker = Locker()
