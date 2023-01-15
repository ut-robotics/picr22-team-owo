
import numpy as np
import cv2

def process_line_images(frag):
    frag_sx, frag_sy = np.shape(frag)
    fragmented_white = np.zeros((frag_sx, frag_sy))
    fragmented_black = np.zeros((frag_sx, frag_sy))
    #print(f"tere hommikust: {frag_sx} {frag_sy}")

    #fragmented_black, fragmented_white = conv_frag(fragmented_black, fragmented_white, frag)
    for y in range(0, frag_sy):
        for x in range(0, frag_sx):
            if frag[y, x] == 6:
                fragmented_black[y, x] = 1
            elif frag[y, x] == 5:
                fragmented_white[y, x] == 1
    
    
    

    return fragmented_white, fragmented_black

def conv_frag(fr_b, fr_w, fr):
    #fragmentedblack[fragmentedblack == 6] = 0
    fr_b[fr == 6] = 1

    #fragmentedwhite[fragmentedwhite == 5] = 0
    fr_w[fr == 5] = 1

    return fr_b, fr_w