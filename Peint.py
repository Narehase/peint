#0.0.1

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

#===================================================================
#
#
# Custom Point Cloud => pointer_list
#
#
# A module that computes a custom point cloud and displays it on an image.
#
#
# When using RPLidar => map_making << Peint.pointer_list
#
#
# Unlike mapmaking, this is an effective module for handling cloud pointer sets.
#
#
#===================================================================




