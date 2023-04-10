from pynput.keyboard import Key,Listener

def press_on (key) :
    if key.char == 'a':
        print("Hi")
    
def press_off (key):
    if key == Key.esc:
        return False

with Listener(on_press = press_on, on_release = press_off) as listener: # type: ignore
    listener.join()