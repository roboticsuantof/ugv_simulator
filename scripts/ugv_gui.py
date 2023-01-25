#!/usr/bin/python3
#librerias de ROS
import rospy
from heuristic_planners import srv
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from trajectory_msgs.msg import *
import std_msgs.msg

#librerias para la GUI
import tkinter
from tkinter import *
from tkinter import messagebox
from PIL import Image,ImageTk
from tkinter.simpledialog import askstring
import math
import random
from waiting import wait
import time
import threading
import copy



#variables
n_inter_point = 0
n_start_point = 0
n_end_point = 0
del_color=""
global cual
cual=[0,0,0,0,0]
fcont=0
lcont=0
algoritmo=std_msgs.msg.String('astar')
global coord
global orden
orden=False
global alg_actual
global color_list
color_sq_ast="#%02x%02x%02x" % (68, 118, 4)
color_li_ast="#%02x%02x%02x" % (108, 197, 81)
color_sq_cost="#%02x%02x%02x" % (71, 98, 79)
color_li_cost="#%02x%02x%02x" % (82, 173, 156)
color_sq_thet="#%02x%02x%02x" % (27, 73, 101)
color_li_thet="#%02x%02x%02x" % (98, 182, 203)
color_sq_lazy="#%02x%02x%02x" % (147, 125, 100)
color_li_lazy="#%02x%02x%02x" % (215, 175, 112)
color_sq_cos_lazy="#%02x%02x%02x" % (254, 144, 0)
color_li_cos_lazy="#%02x%02x%02x" % (255, 221, 74)
#lista que guarda colores de cuadros y lineas por cada algoritmo
color_list=[[color_sq_ast, color_sq_cost,color_sq_ast,color_sq_thet, color_sq_cost, color_sq_thet,color_sq_lazy,color_sq_cos_lazy,color_sq_cos_lazy,color_sq_lazy],[color_li_ast, color_li_cost,color_li_ast,color_li_thet, color_li_cost, color_li_thet,color_li_lazy,color_li_cos_lazy,color_li_cos_lazy,color_li_lazy]]
alg_actual=std_msgs.msg.String('astar')

#for i in (range(5)):
#    cual[i]=int(0)
#print(cual)

#matriz de coordenadas
     #   [[x],[y],[algoritmo]]
coord = [[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0],["", "", "coststar", "coststar", "coststar", "coststar", "coststar"]]
coord_ord = [[coord[0][0], coord[0][1], 0, 0, 0, 0, 0],[coord[1][0], coord[1][0], 0, 0, 0, 0, 0],["", "", "coststar", "coststar", "coststar", "coststar", "coststar"]]
#Crear la instancia de la ventana y menu tkinter
menu = Tk()
win = Tk()

#cargar imagen en el script
img= (Image.open("test_pv_map.pgm.png"))
logo= (Image.open("ua2r.png"))
#dimensiones de ventana
factor_dim=0.5 #factor de dimension para reducir imagen
ancho = int(img.width*factor_dim)
alto = int(img.height*factor_dim)

pantalla_ancho = win.winfo_screenwidth()-2560  # ancho de la pantalla se le resta 2560 por 2 pantallas extras de 1280 cada una
pantalla_alto = win.winfo_screenheight() # alto de la pantalla
print("ancho de pantalla: ",
    pantalla_ancho)
print("alto de pantalla: ",
    pantalla_alto)
factor_img = 1.0 #factor de dimension de la imagen (entre 0.0 - 1.0)
resolucion = 0.05 #resolucion en la que la imagen fue creada


# Calcular las coordenadas de medias del canvas
#x = (pantalla_ancho/2) - (ancho/2)
#y = (pantalla_alto/2) - (alto/2)


#establecer la geometria de la ventana tkinter
win.geometry('%dx%d+%d+%d' % (ancho, alto, win.winfo_screenwidth()-pantalla_ancho, win.winfo_screenheight()-pantalla_alto))



#crear el canvas
canvas= Canvas(win, width= ancho, height= alto)
canvas.pack()

#redimensionar la imagen del mapa usando el metodo "resize"
resized_x=math.floor(ancho*factor_img)
resized_y=math.floor(alto*factor_img)
print("map resized x: ",
    resized_x)
print("map resized y: ",
    resized_y)
resized_image= img.resize((resized_x,resized_y), Image.ANTIALIAS)
new_image= ImageTk.PhotoImage(resized_image, master=win)

#redimensionar logo del menu
resized_logo= logo.resize((188,100), Image.ANTIALIAS)
new_logo= ImageTk.PhotoImage(resized_logo, master=menu)
#agregar la imagen como item del canvas
#canvas.create_image(0,0, anchor=NW, image=new_image)

#se crea el menu
menu.title("Menu")
menu.geometry("188x319")
menu.resizable(0,0)

#posicionar logo en el menu
ua2r=Label(menu,image=new_logo, bg="white",anchor=CENTER)
ua2r.image=new_logo
ua2r.grid(column=3, row=1, sticky=NW)

#agregar la imagen como item del canvas
canvas.create_image(0,0, anchor=NW, image=new_image)
check1=tkinter.IntVar()
check2=tkinter.IntVar()
check3=tkinter.IntVar()
check4=tkinter.IntVar()
check5=tkinter.IntVar()

def asignar_color(algoritmo,forma):
    #forma: [0]=cuadro, [1]=linea
    algoritmo=algoritmo.data
    forma=forma
    if algoritmo == 'astar':
        print("astar")
        color=color_list[forma][0]
    elif algoritmo == 'costastar':
        print("costastar")
        color=color_list[forma][1]
    elif algoritmo =='astarsafetycost':
        print("astarsafetycost")
        color=color_list[forma][2]
    elif algoritmo =="thetastar":
        print("thetastar")
        color=color_list[forma][3]
    elif algoritmo =="costhetastar":
        print("costhetastar")
        color=color_list[forma][4]
    elif algoritmo =="thetastarsafetycost":
        print("thetastarsafetycost")
        color=color_list[forma][5]
    elif algoritmo =="lazythetastar":
        print("lazythetastar")
        color=color_list[forma][6]
    elif algoritmo =="costlazythetastar":
        print("costlazythetastar")
        color=color_list[forma][7]
    elif algoritmo =="costlazythetastarmodified":
        print("costlazythetastarmodified")
        color=color_list[forma][8]
    elif algoritmo =="lazythetastarsafetycost":
        print("lazythetastarsafetycost")
        color=color_list[forma][9]
    
    return color

def proceso_listo(proceso):
    if proceso.ready():
        return True
    return False

def start():
   global n_start_point
   
   if n_start_point < 1:
     init_x = int(askstring('ingresar punto', 'agregar latitud'))
     init_y = int(askstring('ingresar punto', 'agregar longitud'))
     x = int((init_x*factor_dim)/resolucion)
     y = int(resized_y-((init_y*factor_dim)/resolucion))
     y2 =y
     canvas.create_rectangle(x+6,y2+6,x-6,y2-6, outline='blue', fill= 'green')
     print("punto de partida establecido en %d, %d" %(x/factor_dim,((resized_y-y2)/factor_dim)))
     n_start_point+=1
     coord[0][0]=x/factor_dim
     coord[1][0]=((resized_y-y2)/factor_dim)
     print("matriz de coordenadas")
     print("X: ",coord[0])
     print("Y: ",coord[1])
   else:
     print("ya se ingreso el punto de partida")

def end():
   global n_end_point
   
   if n_end_point < 1:
     init_x = int(askstring('ingresar punto', 'agregar latitud'))
     init_y = int(askstring('ingresar punto', 'agregar longitud'))
     x = int((init_x*factor_dim)/resolucion)
     y = int(resized_y-((init_y*factor_dim)/resolucion))
     y2 =y
     canvas.create_rectangle(x+6,y2+6,x-6,y2-6, outline='blue', fill= 'red')
     print("punto de llegada establecido en %d, %d" %(x/factor_dim,((resized_y-y2)/factor_dim)))
     n_end_point+=1
     coord[0][1]=x/factor_dim
     coord[1][1]=((resized_y-y2)/factor_dim)
     print("matriz de coordenadas")
     print("X: ",coord[0])
     print("Y: ",coord[1])
   else:
     print("ya se ingreso el punto de termino")

def inter():
   global n_inter_point
   
   if n_inter_point < 5:
        init_x = int(askstring('ingresar punto', 'agregar latitud'))
        init_y = int(askstring('ingresar punto', 'agregar longitud'))
        x = int((init_x*factor_dim)/resolucion)
        y = int(resized_y-((init_y*factor_dim)/resolucion))
        y2 =y
        canvas.create_rectangle(x+6,y2+6,x-6,y2-6, outline='blue', fill= 'yellow')
        index=n_inter_point +1
        canvas.create_text((x+8),(y+6), text=index,font=('arial', 12, 'bold'))
        print("punto intermedio establecido en %d, %d" %(x/factor_dim,(resized_y-y2)/factor_dim))
        n_inter_point +=1
        print("quedan %d" %(5 - n_inter_point))
        #se busca la posicion vacia
        posFinal_inter=buscar_pos_inter_vacia(coord)
        #hecho=False
        #cont=1
        #while hecho == False:
        #    if coord[0][1+cont] == 0:
        #         if coord[1][1+cont] == 0:
        #                posFinal_inter=cont
        #                hecho=True
        #    cont+=1
        #    print("cont:",cont)
        
            
        coord[0][1+posFinal_inter]=x/factor_dim
        coord[1][1+posFinal_inter]=((resized_y-y2)/factor_dim)
        print("X: ",coord[0])
        print("Y: ",coord[1])
   else: 
        print("se ingreso la cantidad maxima de puntos intermedios")

def buscar_pos_inter_vacia(lista):
    posFinal_inter=0
    hecho=False
    cont=1
    while hecho == False:
        if lista[0][1+cont] == 0:
                if lista[1][1+cont] == 0:
                    posFinal_inter=cont
                    hecho=True
        cont+=1
    return posFinal_inter

def actualizar_n_inter_point(lista):
    cont=0
    for i in range(5):
      if lista[0][2+i] != 0:
                if lista[1][2+i] != 0:
                    cont+=1
    return cont  

def eliminar_point(color):
    global n_start_point
    global n_end_point
    global n_inter_point
    global del_color 
    del_color = color
    
    if del_color == "green":
        x_end=int(coord[0][1]*factor_dim)
        y_end=int(resized_y-(coord[1][1]*factor_dim))
        canvas.create_image(0,0, anchor=NW, image=new_image)

        #se redibuja el punto final
        if n_end_point == 1:
         canvas.create_rectangle((x_end)+6,(y_end)+6,(x_end)-6,(y_end)-6, outline='blue', fill= 'red')
         print("X de punto final: %d" %(x_end))
         print("Y de punto final: %d" %((y_end)))
        else:
         print("no hay punto final")

        #se redibujan los puntos intermedios
        for i in range(5):
            x_inter=int(coord[0][2+i]*factor_dim)
            y_inter=int(coord[1][2+i]*factor_dim)
            canvas.create_rectangle((x_inter)+6,(resized_y-y_inter)+6,(x_inter)-6,(resized_y-y_inter)-6, outline='blue', fill= 'yellow')
            index=i+1
            canvas.create_text((x_inter+8),((resized_y-y_inter)+6), text=index,font=('arial', 12, 'bold'))
            print("x de punto intermedio %d: %d" %(i+1,coord[0][2+i]))
            print("y de punto intermedio %d: %d" %(i+1,coord[1][2+i]))

        print("se eliminó el punto de partida")
        coord[0][0]=0
        coord[1][0]=0
        n_start_point=0
        
    elif del_color == "red":
        x_start=int(coord[0][0]*factor_dim)
        y_start=int(resized_y-(coord[1][0]*factor_dim))
        print(y_start)
        print(x_start)
        canvas.create_image(0,0, anchor=NW, image=new_image)

        #se redibuja el punto inicial
        if n_start_point == 1:
         canvas.create_rectangle((x_start)+6,(y_start)+6,(x_start)-6,(y_start)-6, outline='blue', fill= 'green')
         print("X de punto inicial: %d" %(x_start))
         print("Y de punto inicial: %d" %(resized_y-y_start))
        else:
         print("no hay punto inicial")

        #se redibujan los puntos intermedios
        for i in range(5):
            x_inter=int(coord[0][2+i]*factor_dim)
            y_inter=int(coord[1][2+i]*factor_dim)
            canvas.create_rectangle((x_inter)+6,(resized_y-y_inter)+6,(x_inter)-6,(resized_y-y_inter)-6, outline='blue', fill= 'yellow')
            index=i+1
            canvas.create_text((x_inter+8),((resized_y-y_inter)+6), text=index,font=('arial', 12, 'bold'))
            print("x de punto intermedio %d: %d" %(i+1,coord[0][2+i]))
            print("y de punto intermedio %d: %d" %(i+1,coord[1][2+i]))

        print(" se eliminó el punto de llegada")
        coord[0][1]=0
        coord[1][1]=0
        n_end_point=0

    elif del_color == "yellow":
        opciones_inter =Toplevel(menu)
        opciones_inter.title("eliminar punto")
        #opciones_inter.geometry("188x124")
        #opciones_inter.resizable(0,0)
        
        #global cual
        
        Label(opciones_inter,text= "elija que punto(os) intermedio(os) eliminar").grid(row=0,sticky=W)
        Checkbutton(opciones_inter, text="punto intermedio 1: x:%d,y:%d" %(coord[0][2],coord[1][2]),variable=check1,onvalue=1,offvalue=0, command=check1Clicked).grid(row=1,sticky=W)
        Checkbutton(opciones_inter, text="punto intermedio 2: x:%d,y:%d" %(coord[0][3],coord[1][3]),variable=check2,onvalue=1,offvalue=0, command=check2Clicked).grid(row=2,sticky=W)
        Checkbutton(opciones_inter, text="punto intermedio 3: x:%d,y:%d" %(coord[0][4],coord[1][4]),variable=check3,onvalue=1,offvalue=0, command=check3Clicked).grid(row=3,sticky=W)
        Checkbutton(opciones_inter, text="punto intermedio 4: x:%d,y:%d" %(coord[0][5],coord[1][5]),variable=check4,onvalue=1,offvalue=0, command=check4Clicked).grid(row=4,sticky=W)
        Checkbutton(opciones_inter, text="punto intermedio 5: x:%d,y:%d" %(coord[0][6],coord[1][6]),variable=check5,onvalue=1,offvalue=0, command=check5Clicked).grid(row=5,sticky=W)
        
        Button(opciones_inter, text='eliminar',command=lambda:[eliminar_inter(),opciones_inter.destroy()]).grid(row=6,sticky=W)
        Button(opciones_inter, text='cancelar',command=opciones_inter.destroy).grid(row=7,sticky=W)

        #canvas.create_image(0,0, anchor=NW, image=new_image)

        print("aqui se elimina inter")

    else:
        print("nada que eliminar")

def check1Clicked():
    global cual
    print("lista actual",cual)
    global n_inter_point
    if check1.get() :
        print('punto 1 seleccionado para eliminar')
        cual[0]=1
        #n_inter_point-=1
        print("quedan %d puntos intermedios" %(n_inter_point))
    else :
        print('punto 1 quitado para eliminar')
        cual[0]=0
        #n_inter_point+=1
        print("quedan %d puntos intermedios" %(n_inter_point))

def check2Clicked():
    #global cual
    global n_inter_point
    if check2.get() :
        print('punto 2 seleccionado para eliminar')
        cual[1]=1
        #n_inter_point-=1
        print("quedan %d puntos intermedios" %(n_inter_point))
    else :
        print('punto 2 quitado para eliminar')
        cual[1]=0
        #n_inter_point+=1
        print("quedan %d puntos intermedios" %(n_inter_point))

def check3Clicked():
    #global cual
    global n_inter_point
    if check3.get() :
        print('punto 3 seleccionado para eliminar')
        cual[2]=1
        #n_inter_point-=1
        print("quedan %d puntos intermedios" %(n_inter_point))
    else :
        print('punto 3 quitado para eliminar')
        cual[2]=0
        #n_inter_point+=1
        print("quedan %d puntos intermedios" %(n_inter_point))

def check4Clicked():
    #global cual
    global n_inter_point
    if check4.get() :
        print('punto 4 seleccionado para eliminar')
        cual[3]=1
        #n_inter_point-=1
        print("quedan %d puntos intermedios" %(n_inter_point))
    else :
        print('punto 4 quitado para eliminar')
        cual[3]=0
        #n_inter_point+=1
        print("quedan %d puntos intermedios" %(n_inter_point))

def check5Clicked():
    #global cual
    global n_inter_point
    if check5.get() :
        print('punto 5 seleccionado para eliminar')
        cual[4]=1
        #n_inter_point-=1
        print("quedan %d puntos intermedios" %(n_inter_point))
    else :
        print('punto 5 quitado para eliminar')
        cual[4]=0
        #n_inter_point+=1
        print("quedan %d puntos intermedios" %(n_inter_point))   

req_path=rospy.ServiceProxy('/planner_ros_node/request_path',srv.GetPath,persistent=True)
def eliminar_inter():
    x_start=int(coord[0][0]*factor_dim)
    y_start=int(resized_y-(coord[1][0]*factor_dim))
    x_end=int(coord[0][1]*factor_dim)
    y_end=int(resized_y-(coord[1][1]*factor_dim))
    global cual
    global n_inter_point
    canvas.create_image(0,0, anchor=NW, image=new_image)
    print("matriz de coordenadas")
    print("X: ",coord[0])
    print("Y: ",coord[1])
    print("eliminando puntos medios...")



    for i in range(5):
        print(i)
        print(cual[i])
        if cual[i] == 1:
            print(cual[i])
            x_inter=int(coord[0][2+i]*factor_dim)
            y_inter=int(coord[1][2+i]*factor_dim)
            coord[0][2+i]=0
            coord[1][2+i]=0
            #for j in range(5-i):
            #    if coord[0][2+i+j] != 0:
            #        if coord[1][2+i+j] != 0:
            #                coord[0][2+i]=coord[0][2+i+j]
            #                coord[1][2+i]=coord[1][2+i+j]
            n_inter_point-=1
            print("x de punto intermedio %d: %d" %(i+1,coord[0][2+i]))
            print("y de punto intermedio %d: %d" %(i+1,coord[1][2+i]))
 
    
    for i in range(5):
        if coord[0][2+i] == 0 and coord[1][2+i] == 0:
            for j in range(i,5-i):
               if coord[0][2+j] != 0 and coord[1][2+j] != 0: 
                    coord[0][2+i]=coord[0][2+j]
                    coord[1][2+i]=coord[1][2+j]
                    coord[0][2+j]=0
                    coord[1][2+j]=0
                    print("punto %d cambiado" %(i))
            else:
                print("no hay mas puntos")

    

    print("n_inter_point :",n_inter_point)
    #n_inter_point=actualizar_n_inter_point(coord)
    print("n_inter_point actualizado:",n_inter_point)
    print("puntos intermedios eliminados")
    print("matriz de coordenadas")
    print("X: ",coord[0])
    print("Y: ",coord[1])
    print("redibujando puntos intermedios")
    #se redibujan los puntos intermedios
    vacio=0
    for i in range(5):
     if coord[0][2+i] == 0 and coord[1][2+i] == 0:
        print("interpoint %d vacio" %(i))
        vacio+=1
     else:   
         x_inter=int(coord[0][2+i]*factor_dim)
         y_inter=int(coord[1][2+i]*factor_dim)
         canvas.create_rectangle((x_inter)+6,(resized_y-y_inter)+6,(x_inter)-6,(resized_y-y_inter)-6, outline='blue', fill= 'yellow')
         index=i+1-vacio
         canvas.create_text((x_inter+8),((resized_y-y_inter)+6), text=index,font=('arial', 12, 'bold'))
         print("x de punto intermedio %d: %d" %(i+1,coord[0][2+i]))
         print("y de punto intermedio %d: %d" %(i+1,coord[1][2+i]))    
    #se redibuja el punto inicial
    if n_start_point == 1:
        canvas.create_rectangle((x_start)+6,(y_start)+6,(x_start)-6,(y_start)-6, outline='blue', fill= 'green')
        print("X de punto inicial: %d" %(x_start))
        print("Y de punto inicial: %d" %(resized_y-y_start))
    else:
        print("no hay punto inicial")
        
    #se redibuja el punto final
    if n_end_point == 1:
        canvas.create_rectangle((x_end)+6,(y_end)+6,(x_end)-6,(y_end)-6, outline='blue', fill= 'red')
        print("X de punto final: %d" %(x_end))
        print("Y de punto final: %d" %((y_end)))
    else:
        print("no hay punto final")
    #print(cual)    
            

def eliminar():
    opciones =Toplevel(menu)
    opciones.title("eliminar punto")
    opciones.geometry("188x124")
    opciones.resizable(0,0)
    
    b_start = Button(opciones, bg="white", text="punto de partida", width=20, command=lambda m="green": eliminar_point(m))
    b_start.grid(column=3, row=1)
    b_end = Button(opciones, bg="white", text="punto de termino", width=20, command=lambda m="red": eliminar_point(m))
    b_end.grid(column=3, row=2)
    b_inter = Button(opciones, bg="white", text="punto intermedio", width=20, command=lambda m="yellow": eliminar_point(m))
    b_inter.grid(column=3, row=3)
    b_exit = Button(opciones, bg="white", text="cancelar", width=20, command= opciones.destroy)
    b_exit.grid(column=3, row=4)

def ros_msg():
    data=rospy.wait_for_message('/ugv_path',MultiDOFJointTrajectory,tcp_nodelay=True)
    calc_trayecto.recibir_trayecto(markers= data)
    global rosOk
    rosOk = True

#def background_calculation(markers,n_markers,colorval,puntos):  
    
class calc_trayecto():

    def __init__(self):
    
        # iniciamos el nodo suscriptor.
        # se recibe el mensaje de tipo MultiDOFJointTrajectory()

        #self.mandar_trayecto
        self.image_sub = rospy.Subscriber("/ugv_path", MultiDOFJointTrajectory, self.recibir_trayecto,tcp_nodelay=True)
        #thread = threading.Thread(target=ros_msg)
        #thread.start()
        #time.sleep(0.01)
        #rospy.sleep(1)
        #self.image_sub.unregister()
        
        #data=rospy.wait_for_message('/ugv_path',MultiDOFJointTrajectory)
        #self.recibir_trayecto()

    

    def recibir_trayecto(self,markers):
        print("tramo recibido")
        global lcont
        global puntos
        global colorval
        global alg_actual
        #rospy.sleep(3)
        #print("se esperaron 3 segundos")
           #markers=rospy.wait_for_message('/planner_ros_node/path_points_markers',Marker)
        
        puntos=[[],[]]
        n_markers=len(markers.points)
        r = random.randint(0,255)
        g = random.randint(0,255)
        b = random.randint(0,255)
        #r=50
        #g=100
        #b=200
        rgb = [r,g,b]
        rgb1 =[random.randint(0,255),random.randint(0,255),random.randint(0,255)]
        #colorval="#%02x%02x%02x" % (rgb[0], rgb[1], rgb[2])
        #colorval1="#%02x%02x%02x" % (rgb1[0], rgb1[1], rgb1[2])
        colorval=asignar_color(algoritmo=alg_actual,forma=0)
        print(colorval)
        colorval1=asignar_color(algoritmo=alg_actual,forma=1)
        print(colorval1)
        #thread=threading.Thread(target=background_calculation(markers,n_markers,colorval,puntos))
        #print("for de lineas:",lcont)
        #thread.start()

        for i in range(n_markers):
                
            transforms=markers.points[i].transforms
            translation=transforms[0].translation
            x_mark = translation.x*factor_dim/resolucion
            y_mark = resized_y-(translation.y*factor_dim/resolucion)
            puntos[0].append([x_mark])
            puntos[1].append([y_mark])
            #canvas.create_rectangle((x_mark,y_mark)*2, outline=colorval, fill=colorval)
            #print("x:",x_mark)
            #print("y:",y_mark)
            #time.sleep(0.001)
        for i in range(n_markers):

            if i != 0:
                canvas.create_line(puntos[0][i-1],puntos[1][i-1],puntos[0][i],puntos[1][i],width=2,fill=colorval1)
            canvas.create_rectangle((puntos[0][i],puntos[1][i])*2, outline=colorval, fill=colorval,width=2)

        global num_mark
        global listo 
        num_mark = n_markers
        listo =True


            #print("x:",x_mark)
            #print("y:",y_mark)
            #time.sleep(0.01)
        lcont+=1

#def ros_service_tramo():

def ros_service():
    global orden
    global coord_ord
    global coord
    #coord_final=[]
    #se crea la instancia del servicio /planner_ros_node/request_path
    #req_path=rospy.ServiceProxy('/planner_ros_node/request_path',srv.GetPath,persistent=True)
    print("matriz ordenada:",coord_ord)
    print("matriz no ordenada:", coord)
    if orden == True:
        coord_final=copy.deepcopy(coord_ord)
        print("matriz ordenada")
    else:
        coord_final=copy.deepcopy(coord)
        print("matriz sin ordenar")

    print("coord[0][0]:",coord_final[0][0])
    global fcont
    
    s_x=coord_final[0][0]*resolucion
    s_y=coord_final[1][0]*resolucion
    print("start x:",s_x)
    print("start y:",s_y)

    g_x=coord_final[0][1]*resolucion
    g_y=coord_final[1][1]*resolucion
    print("goal x:",g_x)
    print("goal y:",g_y)

    start_point=Point(s_x,s_y,0)
    goal_point=Point(g_x,g_y,0)
    if coord_final[0][0] == 0 and coord_final[1][0] == 0:
        messagebox.showerror(message="no existe punto de partida")
    if coord_final[1][0] == 0 and coord_final[1][0] == 0:
       messagebox.showerror(message="no existe punto de termino")
    else:
    #se solicita al servicio generar el path de un tramo
       if n_inter_point == 0:
            listo=req_path(start=start_point,goal=goal_point,algorithm=algoritmo)
            

            #wait(lambda: listo,waiting_for="esperando proceso...")
       else:
            print("aqui se dibuja con inter_points")
            desde=start_point
            hasta=Point()
            print("for numero:",fcont)
            for i in range(n_inter_point):
                print("escribiendo: ",i)
                i_x=coord_final[0][2+i]*resolucion
                i_y=coord_final[1][2+i]*resolucion
                hasta=Point(i_x,i_y,0) 
                listo=req_path(start = desde, goal = hasta, algorithm = coord[2][1+i])
                wait(lambda: listo,waiting_for="esperando proceso...")
                desde=hasta
                req_path.close
                #dib=calc_trayecto()
            fcont+=1
            hasta=goal_point
            req_path(start = desde, goal = hasta, algorithm = coord[2][0])
            #dib=calc_trayecto()
    return True
            
def mostrar_matriz():
    print(coord)
    print("partida: x:%d, y:%d" %(coord[0][0], coord[1][0]))
    print("llegada: x:%d, y:%d" %(coord[0][1], coord[1][1]))
    for i in range(5):
        print("inter %d: x:%d, y:%d" %(i+1,coord[0][2+i], coord[1][2+i]))   
    print("n_inter_points:",n_inter_point)
    print("algoritmos:",coord[2])

def set_algoritmo(listbox):
    global algoritmo
    print("entre")
    algoritmo=std_msgs.msg.String(listbox.get(listbox.curselection()[0]))
    print(algoritmo)
    for i in range(n_inter_point+2):
        if len(listbox.curselection())!=0:
            coord[2][i] = algoritmo
    ros=ros_service()
    wait(lambda: ros,waiting_for="esperando proceso...")
    #ros=ros_service()

def set_algoritmo_tramo(listbox,i):
    global orden
    global coord
    global coord_ord 
    if orden == True:
        coord_final=copy.deepcopy(coord_ord)
        print("matriz ordenada")
    else:
        coord_final=copy.deepcopy(coord)
        print("matriz sin ordenar")
    algoritmo=std_msgs.msg.String(listbox.get(listbox.curselection()[0]))
    if len(listbox.curselection())!=0:
        coord[2][i] = algoritmo
        global alg_actual
        alg_actual = algoritmo
        s_x=0
        s_y=0
        if i == 0: 
            global pos
            pos=buscar_pos_inter_vacia(coord_final)
            pos=pos
            print(coord_final)
            print("posicion vacia:",pos)
            s_x=coord_final[0][pos]*resolucion
            s_y=coord_final[1][pos]*resolucion
            print(s_x)
            print(s_y)
            g_x=coord_final[0][1]*resolucion
            g_y=coord_final[1][1]*resolucion
        elif i == 1:
            s_x=coord_final[0][0]*resolucion
            s_y=coord_final[1][0]*resolucion
            g_x=coord_final[0][2]*resolucion
            g_y=coord_final[1][2]*resolucion
        else:
            s_x=coord_final[0][i]*resolucion
            s_y=coord_final[1][i]*resolucion
            g_x=coord_final[0][i+1]*resolucion
            g_y=coord_final[1][i+1]*resolucion
        print("indice:", i)
        start_p=Point(s_x,s_y,0)
        print("start:%d,%d"%(s_x,s_y))
        
        goal_p=Point(g_x,g_y,0)
        print("goal:%d,%d"%(goal_p.x,goal_p.y))

        req_path(start=start_p,goal=goal_p,algorithm=algoritmo)
            

    #ros=ros_service()
    #wait(lambda: ros,waiting_for="esperando proceso...")
    #ros=ros_service()

def select_tramo(tramos, index):
    if tramos:
        opciones_alg =Toplevel(menu)
        opciones_alg.title("elegir tramos")
        opciones_alg.grid_rowconfigure(1,minsize=2)
        listbox=tkinter.Listbox(opciones_alg,font=("Arial",22),height=10)
        listbox.grid(column=0,row=0,rowspan=10,ipadx=7, ipady=1)
        listbox.insert(0,"astar")
        listbox.insert(1,"costastar")
        listbox.insert(2,"astarsafetycost")
        listbox.insert(3,"thetastar")
        listbox.insert(4,"costhetastar")
        listbox.insert(5,"thetastarsafetycost")
        listbox.insert(6,"lazythetastar")
        listbox.insert(7,"costlazythetastar")
        listbox.insert(8,"costlazythetastarmodified")
        listbox.insert(9,"lazythetastarsafetycost")
        label10=tkinter.Label(opciones_alg,text="A*\n\nCost Aware A*\n\nA* Safety Cost\n\nTheta*\n\nCost Aware Theta*\n\nTheta* Safety Cost\n\nLazyTheta*\n\nCost Aware LazyTheta*\n\nCost Aware LazyTheta*\n\nLazyTheta* Safety Cost",font=("Arial",11, "bold"))
        label10.grid(column=4,row=0,rowspan=10,ipadx=3)
        label14=tkinter.Label(opciones_alg,text="-->\n\n-->\n\n-->\n\n-->\n\n-->\n\n-->\n\n-->\n\n-->\n\n-->\n\n-->",font=("Arial",11,"bold"))
        label14.grid(column=3,row=0,rowspan=10)
        label13=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][0])
        label13.grid(column=1,row=0)
        label15=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][0])
        label15.grid(column=2,row=0)
        label16=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][1])
        label16.grid(column=1,row=1)
        label17=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][1])
        label17.grid(column=2,row=1)
        label18=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][2])
        label18.grid(column=1,row=2)
        label19=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][2])
        label19.grid(column=2,row=2)
        label20=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][3])
        label20.grid(column=1,row=3)
        label21=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][3])
        label21.grid(column=2,row=3)
        label22=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][4])
        label22.grid(column=1,row=4)
        label23=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][4])
        label23.grid(column=2,row=4)
        label24=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][5])
        label24.grid(column=1,row=5)
        label25=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][5])
        label25.grid(column=2,row=5)
        label26=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][6])
        label26.grid(column=1,row=6)
        label27=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][6])
        label27.grid(column=2,row=6)
        label28=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][7])
        label28.grid(column=1,row=7)
        label29=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][7])
        label29.grid(column=2,row=7)
        label30=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][8])
        label30.grid(column=1,row=8)
        label31=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][8])
        label31.grid(column=2,row=8)
        label32=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][9])
        label32.grid(column=1,row=9)
        label33=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][9])
        label33.grid(column=2,row=9)

        print(coord)
        n=index+1
        print(n)
        #nlabel=IntVar(value=n)
        #n_alg="n:%d" %(nlabel)

        if n == n_inter_point+1:
            boton=tkinter.Button(opciones_alg,text="seleccionar",command=lambda:[set_algoritmo_tramo(listbox,0),opciones_alg.destroy()])
            boton.grid(column=4,row=11)
            label11=tkinter.Label(opciones_alg,text="seleccione un algoritmo\n para el ultimo tramo")
            label11.grid(column=0,row=10,rowspan=2)
            #label12=tkinter.Label(opciones_alg,textvariable=nlabel)
            #label12.grid(column=1,row=10)
        else:
            boton=tkinter.Button(opciones_alg,text="seleccionar",command=lambda:[set_algoritmo_tramo(listbox,n),opciones_alg.destroy()])
            boton.grid(column=4,row=11)
            label11=tkinter.Label(opciones_alg,text="seleccione un algoritmo\n para el tramo:")
            label11.grid(column=0,row=10,rowspan=2)
            label12=tkinter.Label(opciones_alg,text=n,font=("Arial",22,"bold"))
            label12.grid(column=1,row=11)
        #listbox.mainloop()
        #return True
    else:
        opciones_alg =Toplevel(menu)
        opciones_alg.title("elegir tramos")
        opciones_alg.grid_rowconfigure(1,minsize=2)
        listbox=tkinter.Listbox(opciones_alg,font=("Arial",22),height=10)
        listbox.grid(column=0,row=0,rowspan=10,ipadx=7, ipady=1)
        listbox.insert(0,"astar")
        listbox.insert(1,"costastar")
        listbox.insert(2,"astarsafetycost")
        listbox.insert(3,"thetastar")
        listbox.insert(4,"costhetastar")
        listbox.insert(5,"thetastarsafetycost")
        listbox.insert(6,"lazythetastar")
        listbox.insert(7,"costlazythetastar")
        listbox.insert(8,"costlazythetastarmodified")
        listbox.insert(9,"lazythetastarsafetycost")
        label10=tkinter.Label(opciones_alg,text="A*\n\nCost Aware A*\n\nA* Safety Cost\n\nTheta*\n\nCost Aware Theta*\n\nTheta* Safety Cost\n\nLazyTheta*\n\nCost Aware LazyTheta*\n\nCost Aware LazyTheta*\n\nLazyTheta* Safety Cost",font=("Arial",11, "bold"))
        label10.grid(column=4,row=0,rowspan=10,ipadx=3)
        label14=tkinter.Label(opciones_alg,text="-->\n\n-->\n\n-->\n\n-->\n\n-->\n\n-->\n\n-->\n\n-->\n\n-->\n\n-->",font=("Arial",11,"bold"))
        label14.grid(column=3,row=0,rowspan=10)
        label13=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][0])
        label13.grid(column=1,row=0)
        label15=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][0])
        label15.grid(column=2,row=0)
        label16=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][1])
        label16.grid(column=1,row=1)
        label17=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][1])
        label17.grid(column=2,row=1)
        label18=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][2])
        label18.grid(column=1,row=2)
        label19=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][2])
        label19.grid(column=2,row=2)
        label20=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][3])
        label20.grid(column=1,row=3)
        label21=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][3])
        label21.grid(column=2,row=3)
        label22=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][4])
        label22.grid(column=1,row=4)
        label23=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][4])
        label23.grid(column=2,row=4)
        label24=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][5])
        label24.grid(column=1,row=5)
        label25=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][5])
        label25.grid(column=2,row=5)
        label26=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][6])
        label26.grid(column=1,row=6)
        label27=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][6])
        label27.grid(column=2,row=6)
        label28=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][7])
        label28.grid(column=1,row=7)
        label29=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][7])
        label29.grid(column=2,row=7)
        label30=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][8])
        label30.grid(column=1,row=8)
        label31=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][8])
        label31.grid(column=2,row=8)
        label32=tkinter.Label(opciones_alg,text="\u25A0",fg=color_list[0][9])
        label32.grid(column=1,row=9)
        label33=tkinter.Label(opciones_alg,text="\u2501\u2501",fg=color_list[1][9])
        label33.grid(column=2,row=9)
        #press=False
        boton=tkinter.Button(opciones_alg,text="seleccionar",command=lambda:[set_algoritmo(listbox),opciones_alg.destroy()])
        boton.grid(column=4,row=11)
        label11=tkinter.Label(opciones_alg,text="seleccione un algoritmo para el trayecto")
        label11.grid(column=0,row=10,rowspan=2)
        
        #listbox.mainloop()
        #return press

def mandar_trayecto():
    global coord
    global coord_ord
    global orden
    #algoritmo='astar'
    distancias=[]
    canvas.create_image(0,0, anchor=NW, image=new_image)
    #TODO convertir en funcion el codigo repetido para dibujar todos los puntos
    x_start=int(coord[0][0]*factor_dim)
    y_start=int(resized_y-(coord[1][0]*factor_dim))
    x_end=int(coord[0][1]*factor_dim)
    y_end=int(resized_y-(coord[1][1]*factor_dim))
    #se redibujan los puntos intermedios
    for i in range(5):
         x_inter=int(coord[0][2+i]*factor_dim)
         y_inter=int(coord[1][2+i]*factor_dim)
         canvas.create_rectangle((x_inter)+6,(resized_y-y_inter)+6,(x_inter)-6,(resized_y-y_inter)-6, outline='blue', fill= 'yellow')
         index=i+1
         canvas.create_text((x_inter+8),((resized_y-y_inter)+6), text=index,font=('arial', 12, 'bold'))
         print("x de punto intermedio %d: %d" %(i+1,coord[0][2+i]))
         print("y de punto intermedio %d: %d" %(i+1,coord[1][2+i]))    
    #se redibuja el punto inicial
    if n_start_point == 1:
        canvas.create_rectangle((x_start)+6,(y_start)+6,(x_start)-6,(y_start)-6, outline='blue', fill= 'green')
        print("X de punto inicial: %d" %(x_start))
        print("Y de punto inicial: %d" %(resized_y-y_start))
    else:
        print("no hay punto inicial")
        
    #se redibuja el punto final
    if n_end_point == 1:
        canvas.create_rectangle((x_end)+6,(y_end)+6,(x_end)-6,(y_end)-6, outline='blue', fill= 'red')
        print("X de punto final: %d" %(x_end))
        print("Y de punto final: %d" %((y_end)))
    else:
        print("no hay punto final")

    algoritmo_tramos=False
    sort_inter_point=False

    if n_inter_point != 0:
        sort_inter_point= messagebox.askyesno( "modo de trayecto", "¿activar planificacion dinamica?")
    #se ordenan los inter points

    if sort_inter_point:
        print("antes: ",coord_ord)
        coord_ord=copy.deepcopy(coord)
        print("despues: ",coord_ord)
        for i in range(5):
         x_inter=int(coord_ord[0][2+i])
         y_inter=int(coord_ord[1][2+i])
         distancia=math.sqrt((x_inter-coord_ord[0][0])**2+(y_inter-coord_ord[1][0])**2)
         distancias.append(distancia)
        print("x_ord:",coord_ord[0])
        print("y_ord:",coord_ord[1])
        print("x:",coord[0])
        print("y:",coord[1])
        print("d:",distancias)
        distancias_ordenadas=sorted(distancias)
        print("d_o:",distancias_ordenadas)
        print("d:",distancias)  
        x_aux=0
        y_aux=0 
        d_aux=0
        for i in range(n_inter_point):
            for j in range(n_inter_point):
                if distancias_ordenadas[i] == distancias[j]:
                    d_aux=distancias[i]
                    distancias[i]=distancias[j]
                    distancias[j]=d_aux
                    print(distancias_ordenadas[i])
                    print("i:",i)
                    print("j:",j)
                    x_aux=int(coord_ord[0][2+i])
                    y_aux=int(coord_ord[1][2+i])
                    print("x_aux=",coord_ord[0][2+i])
                    print("y_aux=",coord_ord[1][2+i])
                    coord_ord[0][2+i]=coord_ord[0][2+j]
                    coord_ord[1][2+i]=coord_ord[1][2+j]
                    print("coord_ord[0][2+%d]=%f" %(i,coord_ord[0][2+j]))
                    print("coord_ord[1][2+%d]=%f" %(i,coord_ord[1][2+j]))
                    coord_ord[0][2+j]=x_aux
                    coord_ord[1][2+j]=y_aux
                    print("coord_ord[0][2+%d]=%f" %(j,x_aux))
                    print("coord_ord[1][2+%d]=%f" %(j,y_aux))
                    print("x:",coord_ord[0])
                    print("y:",coord_ord[1])
        print("x ordenada:",coord_ord[0])
        print("y ordenada:",coord_ord[1]) 
        orden=True
    else:
        orden=False
    if n_inter_point != 0:     
        algoritmo_tramos= messagebox.askyesno( "modo de trayecto", "¿desea elegir un algoritmo por cada tramo?")
        if algoritmo_tramos:
            tramo=True
            #actual=0
            for i in range(n_inter_point+1):
                global actual
                actual=select_tramo(tramo,i)
                
                #wait(lambda: actual,waiting_for="esperando proceso...")
            #ros=ros_service()
            #wait(lambda: ros,waiting_for="esperando proceso...")
            #wait(lambda: se_ordena,timeout_seconds=30,waiting_for="esperando proceso...")
        else:
            tramo=False
            select_tramo(tramo,0)
            #wait(lambda: se_ordena,timeout_seconds=30,waiting_for="esperando proceso...")
    else:
        tramo=False
        select_tramo(tramo,0)
        #wait(lambda: se_ordena,timeout_seconds=30,waiting_for="esperando proceso...")
    
def redibujar():
    ros_service()

def redibujar1():
    if n_start_point < 1 or n_end_point < 1:
        print("faltan puntos")
    else:
        r = random.randint(0,255)
        g = random.randint(0,255)
        b = random.randint(0,255)
        #r=50
        #g=100
        #b=200
        rgb = [r,g,b]
        rgb1 =[random.randint(0,255),random.randint(0,255),random.randint(0,255)]
        colorval="#%02x%02x%02x" % (rgb[0], rgb[1], rgb[2])
        colorval1="#%02x%02x%02x" % (rgb1[0], rgb1[1], rgb1[2])
        global num_mark
        global puntos
        #global colorval
        for i in range(num_mark):
                canvas.create_line(puntos[0][i-1],puntos[1][i-1],puntos[0][i],puntos[1][i],width=2,fill=colorval1)
                canvas.create_rectangle((puntos[0][i],puntos[1][i])*2, outline=colorval, fill=colorval)
                print("numero", i)
                #time.sleep(0.01)

       
    

pos_init=Button(menu, bg="white", text="ingresar partida", width=20,command=start,justify=LEFT)
pos_init.grid(column=3, row=2,sticky=NW)
pos_end=Button(menu, bg="white", text="ingresar final", width=20,command=end)
pos_end.grid(column=3, row=3,sticky=NW)
pos_inter=Button(menu, bg="white", text="ingresar punto intermedio", width=20,command=inter)
pos_inter.grid(column=3, row=4,sticky=NW)
pos_del=Button(menu, bg="white", text="eliminar punto", width=20,command=eliminar)
pos_del.grid(column=3, row=5,sticky=NW)
pos_del=Button(menu, bg="white", text="calcular trayecto", width=20,command=mandar_trayecto)
pos_del.grid(column=3, row=6,sticky=NW)
pos_del=Button(menu, bg="white", text="mostrar matriz de trayecto", width=20,command=mostrar_matriz)
pos_del.grid(column=3, row=7,sticky=NW)
pos_del=Button(menu, bg="white", text="redibujar", width=20,command=ros_service)
pos_del.grid(column=3, row=8,sticky=NW)

#definir eventos
def clickIzq(event):
    global n_start_point
    x=int(event.x*resolucion/factor_dim)
    y=int((resized_y-int(event.y))*resolucion/factor_dim)
    x_point=int((x*factor_dim)/resolucion)
    y_point=int(resized_y-(y*factor_dim)/resolucion)
    confirm= False
    print("click izquierdo")

    if n_start_point < 1:
        confirm= messagebox.askyesno( "definir punto", "desea establecer un punto de partida en %d,%d ?" %(x,y))   
        if confirm:
         canvas.create_rectangle(x_point+6,y_point+6,x_point-6,y_point-6, outline='green', fill= 'green')
         print("punto de partida establecido en %d, %d" %(x_point/factor_dim,(resized_y-y_point)/factor_dim))
         n_start_point+=1
         coord[0][0]=x_point/factor_dim
         coord[1][0]=int((resized_y-y_point)/factor_dim)
         print(coord[0])
         print(coord[1])
        else:
          print("punto de partida no establecido")  
    else: 
        print("ya se ingreso el punto de partida")

def clickDer(event):
    global n_end_point
    x=int(event.x*resolucion/factor_dim)
    y=int((resized_y-int(event.y))*resolucion/factor_dim)
    x_point=int((x*factor_dim)/resolucion)
    y_point=int(resized_y-((y*factor_dim)/resolucion))
    confirm= False
    print("click derecho")
    
    if n_end_point < 1:
        confirm= messagebox.askyesno( "definir punto", "desea establecer un punto de llegada en %d,%d ?" %(x,y))
        if confirm:
         canvas.create_rectangle(x_point+6,y_point+6,x_point-6,y_point-6, outline='red', fill= 'red')
         print("punto de partida establecido en %d, %d" %(x_point/factor_dim,y/resolucion))
         coord[0][1]=x_point/factor_dim
         coord[1][1]= int(y/resolucion)
         n_end_point+=1
         print(coord[0])
         print(coord[1])
        else:
         print("punto de partida no establecido")
    else: 
     print("ya se ingreso el punto de termino")

def clickScroll(event):
    global n_inter_point
    x=int(event.x*resolucion/factor_dim)
    y=int((resized_y-int(event.y))*resolucion/factor_dim)
    x_point=int((x*factor_dim)/resolucion)
    y_point=int(resized_y-((y*factor_dim)/resolucion))
    confirm= False
    print("click scroll")
    
    

    if n_inter_point < 5:
        confirm= messagebox.askyesno( "definir punto", "desea establecer un punto intermedio en %d,%d ?" %(x,y))

        if confirm: 
            canvas.create_rectangle(x_point+6,y_point+6,x_point-6,y_point-6, outline='yellow', fill= 'yellow')
            index=n_inter_point+1
            canvas.create_text((x_point+8),(y_point+6), text=index,font=('arial', 12, 'bold'))
            print("punto de partida establecido en %d, %d" %(x_point/factor_dim,y/resolucion))
            print("quedan %d" %(5 - n_inter_point))
            #se busca la posicion vacia
            posFinal_inter=0
            hecho=False
            cont=1
            while hecho == False:
                if coord[0][1+cont] == 0:
                    if coord[1][1+cont] == 0:
                            posFinal_inter=cont
                            hecho=True
                cont+=1
                print("cont:",cont)
            coord[0][1+posFinal_inter]=x_point/factor_dim
            coord[1][1+posFinal_inter]=int(y/resolucion)
            n_inter_point +=1
            print(coord[0])
            print(coord[1])
        else: 
         print("punto de partida no establecido")
    else: 
        print("se ingreso la cantidad maxima de puntos intermedios")

#Set the geometry of tkinter frame
win.geometry('%dx%d' % (ancho, alto))
def callback(e):
   x= e.x
   y= resized_y-e.y
   #canvas.create_rectangle((x+2,resized_y-y+2)*2, outline='red', fill= 'red')
   print("el puntero esta en %d, %d" %(x/factor_dim,y/factor_dim))

win.bind('<Motion>',callback)
win.bind('<Button-1>', clickIzq)
win.bind('<Button-3>', clickDer)
win.bind('<Button-2>', clickScroll)



def mainros():
    rospy.init_node('puente_GUI', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():  
    hola_str = "esperando datos... %s" % rospy.get_time()
    rospy.loginfo(hola_str)
    #cal=calc_trayecto()
        #rate.sleep

if __name__ == '__main__':
    try:
        mainros()
        cal=calc_trayecto()
    except rospy.ROSInterruptException:
        pass

win.mainloop()