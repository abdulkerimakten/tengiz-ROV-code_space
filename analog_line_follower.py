import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FPS, 15)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

def main():
    while True:
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

        command = "STOP"

        if contours:
            # En büyük konturu seç
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M['m00'] > 0:
                # Konturun merkezi
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Konturun merkezine çember ekle
                cv2.circle(frame, (cx, int(cy + height/2)), 10, (0, 255, 0), -1)

                # Görüntü merkezine göre hata
                error = cx - width // 2

                # Motor komutlarını belirleme
                if abs(error) > 100:  # Hata payını artırmak için 50 yerine 100 kullanıldı
                    if error > 100:  # 100 pikselden fazla sağa kayma
                        command = 'MOVE_LEFT'
                    elif error < -100:  # 100 pikselden fazla sola kayma
                        command = 'MOVE_RIGHT'
                else:
                    command = 'FORWARD'

                # Keskin dönüş kontrolü
                rect = cv2.minAreaRect(largest_contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

                angle = rect[-1]
                if angle < -45:
                    angle += 90

                if abs(angle) > 45:
                    if angle > 0:
                        command = 'TURN_RIGHT'
                    else:
                        command = 'TURN_LEFT'
                
        else:
            command = 'STOP'

        # Komutu görüntüde göster
        cv2.putText(frame, command, (width//2, height//2 + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # Çerçeveyi göster
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
