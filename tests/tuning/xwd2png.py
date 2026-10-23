import struct, sys, numpy as np, cv2
d=open(sys.argv[1],"rb").read()
h=struct.unpack(">25I", d[:100]); hsize,w,ht,bpl,nc=h[0],h[4],h[5],h[12],h[19]
img=np.frombuffer(d[hsize+nc*12:hsize+nc*12+bpl*ht],dtype=np.uint8).reshape(ht,bpl)[:,:w*4].reshape(ht,w,4)[:,:,:3]
cv2.imwrite(sys.argv[2], cv2.resize(img,(w*2//3,ht*2//3)))
