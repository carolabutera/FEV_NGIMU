import time
from datetime import datetime
import os 
import csv
import PySimpleGUI as sg

sg.theme("LightGreen2")
layout_subj_info = [[sg.Text("Insert Subject ID", font=('Arial',12))],    
            [sg.Input()],
            [sg.Text("Insert Subject AGE", font=('Arial',12))],
            [sg.Input()],
            [sg.Text("Insert Subject HEIGHT", font=('Arial',12))],
            [sg.Input()],
            [sg.Text("Sex:", font=('Arial',12))],
            [sg.OptionMenu(["M","F"])],
            [sg.Text("Dominant arm:", font=('Arial',12))],
            [sg.OptionMenu(["L","R"])],
            [sg.Button('Ok', font=('Arial',12))]]


#window to ask subject information 
window_subj_info= sg.Window("Data acquisition protocol", layout_subj_info, margins=(200,150))

event, values = window_subj_info.read()                   

# Do something with the information gathered
IDs=values[0]
age=values[1]
height=values[2]
sex=values[3]
dom_arm=values[4]


print(IDs,age,height,dom_arm,sex)
window_subj_info.close() 


name_subject=IDs+"_SUBJ_INFO"+".csv"
name_raw_data= "RAW_"+IDs+"_"".csv"

outdir = './DATA'
if not os.path.exists(outdir):
    os.mkdir(outdir)

name_subjfile=os.path.join(outdir, name_subject)

subj=open(name_subjfile, 'w', encoding='UTF8', newline='')

writer_subj=csv.writer(subj, delimiter=',')
header_subj=['ID','age','heigth','sex','dom_arm']

writer_subj.writerow(header_subj)

calib_path = os.path.join(outdir, IDs+"_calibration.csv") 
calib=open(calib_path, 'w', encoding='UTF8', newline='')
writer_calib=csv.writer(calib, delimiter=',')

header_calib=['pose','pose_repetition','time','TOxx', 'TOyx','TOzx','TOxy' ,'TOyy', 'TOzy','TOxz' ,'TOyz' ,'TOzz','UAxx_R', 'UAyx_R','UAzx_R','UAxy_R' ,'UAyy_R', 'UAzy_R','UAxz_R' ,'UAyz_R' ,'UAzz_R','FAxx_R', 'FAyx_R','FAzx_R','FAxy_R' ,'FAyy_R', 'FAzy_R','FAxz_R' ,'FAyz_R' ,'FAzz_R','UAxx_L', 'UAyx_L','UAzx_L','UAxy_L' ,'UAyy_L', 'UAzy_L','UAxz_L' ,'UAyz_L' ,'UAzz_L','FAxx_L', 'FAyx_L','FAzx_L','FAxy_L' ,'FAyy_L', 'FAzy_L','FAxz_L' ,'FAyz_L' ,'FAzz_L', 'a_TO_x','a_TO_y','a_TO_z','n']
writer_calib.writerow(header_calib)

writer_subj.writerow([IDs, age , height,sex,dom_arm])

subj.close()
