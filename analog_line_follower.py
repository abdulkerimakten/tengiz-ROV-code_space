import cv2
import numpy as np

def main():
    cap = cv2.VideoCapture(0)  # Kameradan görüntü almak için
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        # Görüntüyü gri tonlamalıya çevir
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Siyah şeridi tespit etmek için görüntüyü eşikleme
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        
        # Görüntünün alt kısmını kırpma (yolun belirli bir kısmını incelemek için)
        height, width = binary.shape
        crop_img = binary[int(height/2):height, 0:width]
        
        # Kontur bulma
        contours, _ = cv2.findContours(crop_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # En büyük konturu seç
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M['m00'] > 0:
                # Konturun merkezi
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Görüntü merkezine göre hata
                error = cx - width // 2
                
                # Motor komutlarını belirleme
                if error > 50:  # 50 pikselden fazla sağa kayma
                    command = 'TURN_RIGHT'
                elif error < -50:  # 50 pikselden fazla sola kayma
                    command = 'TURN_LEFT'
                else:
                    command = 'FORWARD'
                
                # Çizgiyi bulduğumuzu belirtmek için bir işaretçi çizelim
                cv2.circle(frame, (cx, int(height/2) + cy), 10, (0, 255, 0), -1)
                cv2.putText(frame, f"Error: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                command = 'STOP'
        else:
            command = 'STOP'
        
        # Komutu görüntüde yazdır
        cv2.putText(frame, f"Command: {command}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        
        # Görüntüyü göster
        cv2.imshow('Line Follower', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
