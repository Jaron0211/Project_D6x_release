import tkinter
import select
import numpy as np

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style

logfile = open('fly_log.txt','w+')

root = tkinter.Tk()
root.wm_title("室內定位-dev")

#matplotlib animate
style.use('ggplot')
fig = Figure(figsize = (5,5),dpi = 100)
ani = fig.add_subplot(111)

canvas = FigureCanvasTkAgg(fig, master=root)  # A tk.DrawingArea.
canvas.draw()
canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)

'''
toolbar = NavigationToolbar2Tk(canvas, root)
toolbar.update()
canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)
'''

T = tkinter.Text(root, height = 20, width = 60)

import socket
#HOST = '192.168.50.157'
HOST = '127.0.0.1'
PORT = 8000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)

readable = [server]

xlist = []
ylist = []
zlist = []
alist = []
click = False

def mouse_click(even):
    global click
    click = True
    print("o0k")

def animate(i):
    '''
    pulldata = open('log.txt','r').read()
    datalist = pulldata.split('\n')
    xlist = []
    ylist = []
    zlist = []
    alist = []

    for eachline in datalist:
        if len(eachline)>1:
            x,y,z,a = eachline.split(',')
            xlist.append(int(float(x)))
            ylist.append(int(float(y)))
            zlist.append(int(float(z)))
            alist.append(int(float(a)))
    '''
    tcp_pos_draw()

def tcp_pos_draw():
    x, y, z, a = 0, 0, 0, 0
    r, w, e = select.select(readable, [], [], 0)
    ani.axis([300, -30, -30, 400])
    for rs in r:
        if rs == server:
            conn, addr = server.accept()
            clientMessage = str(conn.recv(1024), encoding='utf-8')
            if(len(list(clientMessage))>0):
                dataset = clientMessage.split('\n')
                global text
                text = clientMessage
                T.delete('1.0', tkinter.END)
                T.insert(tkinter.END, text)
                print(dataset)
                for eachdata in dataset:
                    if('POSITION' in eachdata):
                        datatag = eachdata.split(':')
                        x,y,z,a = datatag[1].split(',')
                        if not (int(float(x)) == 0 and int(float(y)) == 0 and int(float(z)) == 0 and int(float(a)) == 0):
                            xlist.append(float(x))
                            ylist.append(float(y))
                            zlist.append(float(z))
                            alist.append(float(a))
                            logfile.write(clientMessage)
                            logfile.write('\n')
                            print(x, y, z, a)

                            ani.clear()
                            ani.plot(xlist,ylist,c = 'green')
                            #ani.scatter(x, y, s=60, c='green')
                            ani.axis([300, -30, -30, 400])
                            ani.plot(round(float(x),2), round(float(y),2), '-ro')

def on_key_press(event):
    print("you pressed {}".format(event.key))
    key_press_handler(event, canvas, toolbar)

canvas.mpl_connect("key_press_event", on_key_press)


def _quit():
    fig.savefig('fly_log.png')
    root.quit()     # stops mainloop
    root.destroy()  # this is necessary on Windows to prevent
                    # Fatal Python Error: PyEval_RestoreThread: NULL tstate


button = tkinter.Button(master=root, text="Quit", command=_quit)

T.pack()
button.pack(side=tkinter.BOTTOM)
anima = animation.FuncAnimation(fig,animate,interval=500)

tkinter.mainloop()
# If you put root.destroy() here, it will cause an error if the window is
# closed with the window manager.