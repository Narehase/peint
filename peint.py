from math import *


class palet:
    def __init__(self,Rx = 0 ,Ry = 0) -> None:
        self.Rx = Rx
        self.Ry = Ry
        self.rSeta = 0
        pass

    def pointer(self,img,Seta,r):
        sins = r * sin(Seta) + self.Ry
        coss = r * cos(Seta) + self.Rx
        try:
            sub_color = [0,255,0]
            img[sins,coss] = [0,0,255]


            img[sins,coss+1] = sub_color
            img[sins+1,coss] = sub_color

            img[sins,coss-1] = sub_color
            img[sins-1,coss] = sub_color
            #img[sins,coss] = [255,255,255]
        except IndexError:
            pass

        return img

    def pointer_list(self,img,Seta_r_list,xp = 1,run = 0, run_Seta = 0):
        self.rSeta += radians(run_Seta + 90)
        #ry = int (run * cos(self.rSeta) + self.Ry)
        #rx = int (run * sin(self.rSeta) + self.Rx)
        self.Ry = self.Ry + run

        rx = self.Rx
        ry = self.Ry

        print("==========================",len(Seta_r_list))
        for i in range(int(len(Seta_r_list))):
            
            Seta = Seta_r_list[i][0] 
            r = Seta_r_list[i][1] * xp


            #print(Seta," || ", r)



            coss = int(r * sin(radians(Seta + run_Seta)) + rx)
            sins = int(r * cos(radians(Seta + run_Seta))) + ry
                
            try:


                sub_color = [0,255,0]
                img[sins,coss] = [0,0,255]


                img[sins,coss+1] = sub_color
                img[sins+1,coss] = sub_color

                img[sins,coss-1] = sub_color
                img[sins-1,coss] = sub_color
                #img[sins,coss] = [255,255,255]
                iof = 1
                for sam1 in range(10):
                    for Setar in range(720):
                        xs = 20*cos(radians(Setar/2))
                        ys = 20*sin(radians(Setar/2))
                        img[ry + ys,rx + xs] = [255,0,0]    
                    img[ry  + sam1,rx] = [255,0,0]

                    
                    '''for sam2 in range(int(iof)):
                        img[self.Ry - 5 + sam1,self.Rx + sam2] = [255,0,0]
                        
                        img[self.Ry - 5 + sam1,self.Rx - sam2] = [255,0,0]
                    iof += 0.5'''
                    '''
                    cossc = int(sam1+1 * sin(radians(run_Seta+135)) ) + self.Ry
                    sinsc = int(sam1+1 * cos(radians(run_Seta+135)) ) + self.Rx
                    img[sinsc,cossc] = [0,0,255]
                    
                    cossc1 = int(sam1+1 * sin(radians(run_Seta+90)) ) + self.Ry
                    sinsc1 = int(sam1+1 * cos(radians(run_Seta+90)) ) + self.Rx
                    img[sinsc1,cossc1] = [0,255,0]
                    '''
                    #img[self.Ry + sam1,self.Rx] = [255,0,0]
            
            except IndexError:
                continue
                pass
        
        return img

class cals:
    def __init__(self, num = 2) -> None:
        pass
    
    

    def auto_Ri(self,ser,LIDAR):
        def _CheckSum(data):
            try:
                ocs = _HexArrToDec((data[6],data[7]))
                LSN = data[1]
                cs = 0x55AA^_HexArrToDec((data[0],data[1]))^_HexArrToDec((data[2],data[3]))^_HexArrToDec((data[4],data[5]))
                for i in range(0,2*LSN,2):
                    cs = cs^_HexArrToDec((data[8+i],data[8+i+1])) 
                    
                if(cs==ocs):
                    return True
                else:
                    return False
            except Exception as e:
                return False
                
        def _HexArrToDec(data):
            littleEndianVal = 0
            for i in range(0,len(data)):
                littleEndianVal = littleEndianVal+(data[i]*(256**i))
            return littleEndianVal

        def _AngleCorr(dist):
            if dist==0:
                return 0
            else:
                return (atan(21.8*((155.3-dist)/(155.3*dist)))*(180/pi))
                
        def _Calculate(d):
            ddict=[]
            LSN=d[1]
            Angle_fsa = ((_HexArrToDec((d[2],d[3]))>>1)/64.0)
            Angle_lsa = ((_HexArrToDec((d[4],d[5]))>>1)/64.0)
            if Angle_fsa<Angle_lsa:
                Angle_diff = Angle_lsa-Angle_fsa
            else:
                Angle_diff = 360+Angle_lsa-Angle_fsa
            for i in range(0,2*LSN,2):
                dist_i = _HexArrToDec((d[8+i],d[8+i+1]))/4
                Angle_i_tmp = ((Angle_diff/float(LSN))*(i/2))+Angle_fsa
                if Angle_i_tmp > 360:
                    Angle_i = Angle_i_tmp-360
                elif Angle_i_tmp < 0:
                    Angle_i = Angle_i_tmp+360
                else:
                    Angle_i = Angle_i_tmp
                
                Angle_i = Angle_i +_AngleCorr(dist_i)
                ddict.append((dist_i,Angle_i))
            return ddict

        def code(ser):
            ip = []
            data_1 = ser.read(6000)
            #print(data_1)
            
            data_2 = data_1.split(b"\xaa\x55")[1:-1]

            #print(bytearray(data_1))

            
            for i,e in enumerate(data_2):
                #print(i)
                try:
                    if(e[0]==0):
                        if(_CheckSum(e)):
                            d = _Calculate(e)
                            for ele in d:
                                #print(ele[1])
                                angle = (ele[1])
                                if angle >= 0 and angle < 360:
                                    #print(ele[0], " || ", angle)
                                    ip.append([angle,ele[0]])




                except Exception as e:
                    pass
                    print("err")

            return ip

        igp = []
        iop = code(LIDAR)

        for l in iop:
            igp.append([360 - l[0],l[1]])
        op = 0
        for i in igp:
            if int(i[0]) == 180:
                op = i[1]
                break
        


        ser.write("50".encode())
        a = ser.readline().encode()
        if a[:2] == "ok":
            igp = []
            iop = code(LIDAR)

            for l in iop:
                igp.append([360 - l[0],l[1]])
            op1 = 0
            for i in igp:
                if int(i[0]) == 180:
                    op1 = i[1]
                    break

            print(op - op1)




