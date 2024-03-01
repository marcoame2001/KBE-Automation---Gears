#IN ORDER TO RUN THIS CODE YOU MUST INTRODUCE THE PARAMETERS COPING THE TXT TEXT AND PASTING IT INTO THE JOURNAL PARAMETERS SECTION (SEE THE EXPLANATIONS IN THE REPORT)
#IMPORTANT!!! IF YOU PUT WRONG ONE SPACE OR A COMA IT DOES NOT WORK CORRECTLY (SEE THE REPORT)
#IN ORDER TO AVOID ERRORS RESTART THE NX PROGRAM WITH EACH EXAMPLE
#BE CAREFUL WITH THE PATHS
from Shapes.Cylinder import Cylinder
from Shapes.Block import Block
from Motion.Motion import Motion
import random
import NXOpen
import math
import sys
import NXOpen.UF


object_list = []
number_list = [1]
compoused = [False]

#MESSAGE
uf_session = NXOpen.UF.UFSession.GetUFSession()
message = "The following arguments were passed to my journal:" + sys.argv[1] + "," + sys.argv[2] + "," + sys.argv[3] + "," + sys.argv[4] + "," + sys.argv[5] + "," + sys.argv[6] + "," + sys.argv[7] + "."
uf_session.Ui.DisplayMessage(message, 1)

    

#PARAMETERS
input_output_distance_ratio = sys.argv[1]   #distance between the first teeth of the first gear and the las teeth of the last gear

radius_list_p = sys.argv[2] #list of lists, the inside ones are for the Z axis in order to create compounds gears. And outside ones for gears in the X axis
lent =[]
lenr = []
lend =''
for i in radius_list_p:
    if (i != ' ') and (i != ','): 
        lend += i
        
    elif i == ',':
        lenr.append(int(lend))
        lend = ''
        lent.append(lenr)
        lenr =[]
        
    elif i == ' ':

        lenr.append(int(lend))
        lend = ''
lenr.append(int(lend))
lend = ''
lent.append(lenr)
lenr =[]
radius_list = lent

centers_list_p = sys.argv[3]                #center radius for the gears in the X axis (the compounds ones must have the same center radius)
leni = []
leno =''
for i in centers_list_p:
    if (i != ' '): 
        leno += i

    elif i == ' ':

        leni.append(int(leno))
        leno = ''
leni.append(int(leno))
centers_list = leni

x_direction = sys.argv[4]                    #if you want the gears to be generated at the right side or at the left side

z_direction = sys.argv[5]                    #if you want the compounds gears to be generated on top or underneath

height_list_p = sys.argv[6]            #the list of the heightsof each gear (has the same order as the radius_list)
lenu = []
leny =''
for i in height_list_p:
    if (i != ' '): 
        leny += i

    elif i == ' ':

        lenu.append(int(leny))
        leny = ''
lenu.append(int(leny))
height_list = lenu

tooth_length = sys.argv[7]                    #the length of each teeth, you must not change this parameters significantly
direction = sys.argv[8]


class GeneralGear:
    
    def __init__(self, radius, tooth_height, center, x, y, z, direction, cylinder_height):
        self.radius = radius
        self.tooth_height = tooth_height
        self.center = center
        self.x = x
        self.y = y
        self.z = z
        self.direction = direction
        self.cylinder_height = cylinder_height
        
    def generate_gear (self):
        teeth_number = math.floor(self.radius / 0.8)
        angle = 2*math.pi / teeth_number
        num = (teeth_number*2)+number_list[len(number_list)-1]+3
        cylinder_radius = self.radius-self.tooth_height
        gear = Cylinder(self.x,self.y,self.z, cylinder_radius * 2, self.cylinder_height,  [0,0,self.direction])
        if (self.direction == 1):
            for i in range(0, teeth_number):
                tooth = Block(self.x + (cylinder_radius-0.1*self.tooth_height)*math.cos(i*angle), 
                self.y + (cylinder_radius-0.1*self.tooth_height)*math.sin(i*angle),
                self.z,
                self.tooth_height,
                self.tooth_height,
                self.cylinder_height,
                NXOpen.Vector3d(math.cos(i*angle), math.sin(i*angle), 0.0),
                NXOpen.Vector3d(math.cos(i*angle+math.pi/2), math.sin(i*angle+math.pi/2),0.0))
                gear.unite(tooth)
        elif (self.direction == -1):
            for i in range(0, teeth_number):
                tooth = Block(self.x + (cylinder_radius-0.1*self.tooth_height)*math.cos(i*angle), 
                self.y + (cylinder_radius-0.1*self.tooth_height)*math.sin(i*angle),
                self.z-self.cylinder_height,
                self.tooth_height,
                self.tooth_height,
                self.cylinder_height,
                NXOpen.Vector3d(math.cos(i*angle), math.sin(i*angle), 0.0),
                NXOpen.Vector3d(math.cos(i*angle+math.pi/2), math.sin(i*angle+math.pi/2),0.0))
                gear.unite(tooth)
            
        hole = Cylinder(self.x, self.y, self.z, self.center, self.cylinder_height,[0,0,self.direction])
        gear.subtract(hole)
        pop = compoused.pop()
        if pop == True:
            if self.z !=0:
                number_list.pop(len(number_list)-2)
                number_list.append(num+1)
                object_list[len(object_list)-1].unite(gear)
                compoused.append(True)
            else:
                number_list.pop(len(number_list)-2)
                number_list.append(num)
                object_list.append(gear)
                compoused.append(False)
        elif pop == False:
            if self.z !=0:
                number_list.append(num+1)
                object_list[len(object_list)-1].unite(gear)
                compoused.append(bool(True))
            else:
                number_list.append(num)
                object_list.append(gear)
                compoused.append(False)
        return gear
    
    

class Serialize:
    
        def __init__(self,gear_radius_list, center_list, dir_x , dir_z, height_list, tooth_length, input_output_distance_ratio):
            self.gear_radius_list = gear_radius_list
            self.center_list = center_list
            self.dir_x = dir_x
            self.dir_z = dir_z
            self.height_list = height_list
            self.tooth_length = tooth_length
            self.input_output_distance_ratio = input_output_distance_ratio
        
        def fabric(self):
            for i in range(0,len(self.gear_radius_list)):
                if i == 0 :
                    temp_sum = 2*self.gear_radius_list[0][0]
                else:
                    temp_sum += 2*self.gear_radius_list[i][0] - self.tooth_length
            if (temp_sum != self.input_output_distance_ratio):
                    sys.exit("The ratio between input and output gears with the parameters introduced is wrong so the solution will not work, so it will not be generated because some of them will not be interlaced or will be generated in an overlapping way")     
            temp_prev_x = 0
            for i in range(0, len(self.gear_radius_list)):
            
                if(i == 0):
                    if (self.dir_z == 1):
                        temp_prev_z = 0           
                        for j in range(0,len(self.gear_radius_list[i])):
                            
                            if(j==0):
                                center = self.center_list[i]
                                x = 0
                                y = 0
                                z = 0
                                temp_height = 0
                            else:
                                center = self.center_list[i]
                                x = 0
                                y = 0
                                z = temp_prev_z + self.height_list[temp_height]
                                temp_height += 1
                            
                            gear = GeneralGear(self.gear_radius_list[i][j], self.tooth_length, center, x, y, z, 1, self.height_list[temp_height])
                            gear.generate_gear()
                            temp_prev_z = z
                            
                    elif (self.dir_z == -1):
                        temp_prev_z = 0           
                        for j in range(0,len(self.gear_radius_list[i])):
                            
                            if(j==0):
                                center = self.center_list[i]
                                x = 0
                                y = 0
                                z = 0
                                temp_height = 0
                            else:
                                center = self.center_list[i]
                                x = 0
                                y = 0
                                z = temp_prev_z + self.dir_z*(self.height_list[temp_height])
                                temp_height += 1
                            
                            gear = GeneralGear(self.gear_radius_list[i][j], self.tooth_length, center, x, y, z,-1, self.height_list[temp_height])
                            gear.generate_gear()
                            
                            temp_prev_z = z
                        
                    temp_prev_x = x
                
                else:
                    if (self.dir_z == 1):
                        temp_prev_z = 0
                        for j in range(0,len(self.gear_radius_list[i])):
                            
                            if(j==0):
                                center = self.center_list[i]
                                x = temp_prev_x + self.dir_x*(self.gear_radius_list[i][j] + self.gear_radius_list[i-1][j] - self.tooth_length)
                                y = 0
                                z = 0
                                temp_height += 1
                            else:
                                center = self.center_list[i]
                                x = temp_prev_x + self.dir_x*(self.gear_radius_list[i][0] + self.gear_radius_list[i-1][0] - self.tooth_length)
                                y = 0
                                z = temp_prev_z + self.height_list[temp_height]
                                temp_height += 1
                            
                            gear= GeneralGear(self.gear_radius_list[i][j], self.tooth_length, center, x, y, z, 1, self.height_list[temp_height])
                            gear.generate_gear()
                            temp_prev_z = z
                    elif (self.dir_z == -1):
                        temp_prev_z = 0
                        for j in range(0,len(self.gear_radius_list[i])):
                            
                            if(j==0):
                                center = self.center_list[i]
                                x = temp_prev_x + self.dir_x*(self.gear_radius_list[i][j] + self.gear_radius_list[i-1][j] - self.tooth_length)
                                y = 0
                                z = 0
                                temp_height += 1
                                
                            else:
                                center = self.center_list[i]
                                x = temp_prev_x + self.dir_x*(self.gear_radius_list[i][0] + self.gear_radius_list[i-1][0] - self.tooth_length)
                                y = 0
                                z = temp_prev_z + self.dir_z*(self.height_list[temp_height])
                                temp_height += 1
                            
                            gear= GeneralGear(self.gear_radius_list[i][j], self.tooth_length, center, x, y, z, -1, self.height_list[temp_height])
                            gear.generate_gear()
                            temp_prev_z = z
                        
                    temp_prev_x = x
                         
serial = Serialize(radius_list, centers_list, int(x_direction), int(z_direction), height_list, int(tooth_length), int(input_output_distance_ratio))


serial.fabric()

#ANIMATION
pathToTheFolder = "\\\\sambaad.stud.ntnu.no\\alvaroor\\.profil\\stud\\datasal\\Desktop\\assigment2\\final\\Animation\\"
fileName = "geartrain_" + str(random.randint(1,10000)) 
motion = Motion(object_list, 0, 10, 10, 0, True, pathToTheFolder, fileName, number_list, float(direction))