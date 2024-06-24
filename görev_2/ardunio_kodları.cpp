#define FRONT_LEFT_MOTOR_FORWARD_PIN 3
#define FRONT_LEFT_MOTOR_BACKWARD_PIN 4
#define FRONT_RIGHT_MOTOR_FORWARD_PIN 5
#define FRONT_RIGHT_MOTOR_BACKWARD_PIN 6
#define BACK_LEFT_MOTOR_FORWARD_PIN 7
#define BACK_LEFT_MOTOR_BACKWARD_PIN 8
#define BACK_RIGHT_MOTOR_FORWARD_PIN 9
#define BACK_RIGHT_MOTOR_BACKWARD_PIN 10
#define UP_MOTOR_PIN 11
#define DOWN_MOTOR_PIN 12

void setup() {
    Serial.begin(9600);  // Seri iletişimi başlat
    // Motorları başlatmak için gerekli pinleri ayarla
    pinMode(FRONT_LEFT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(FRONT_LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(FRONT_RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(FRONT_RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(BACK_LEFT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(BACK_LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(BACK_RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(BACK_RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(UP_MOTOR_PIN, OUTPUT);
    pinMode(DOWN_MOTOR_PIN, OUTPUT);
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();

        switch(command) {
            case 'F':
                moveForward();
                break;
            case 'L':
                turnLeft();
                break;
            case 'R':
                turnRight();
                break;
            case 'B':
                moveBackward();
                break;
            case 'U':
                moveUp();
                break;
            case 'D':
                moveDown();
                break;
            case 'S':
                stopMotors();
                break;
            default:
                // Bilinmeyen komutlar için hiçbir şey yapma
                break;
        }
    }
}

void moveForward() {
    // Motorları ileri hareket ettir
    digitalWrite(FRONT_LEFT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(FRONT_RIGHT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(FRONT_LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(FRONT_RIGHT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(BACK_LEFT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(BACK_RIGHT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(BACK_LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(BACK_RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

void turnLeft() {
    // Motorları sola döndür
    digitalWrite(FRONT_LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(FRONT_RIGHT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(FRONT_LEFT_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(FRONT_RIGHT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(BACK_LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(BACK_RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(BACK_LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(BACK_RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

void turnRight() {
    // Motorları sağa döndür
    digitalWrite(FRONT_LEFT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(FRONT_RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(FRONT_LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(FRONT_RIGHT_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(BACK_LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(BACK_RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(BACK_LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(BACK_RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

void moveBackward() {
    // Motorları geri hareket ettir
    digitalWrite(FRONT_LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(FRONT_RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(FRONT_LEFT_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(FRONT_RIGHT_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(BACK_LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(BACK_RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(BACK_LEFT_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(BACK_RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}

void moveUp() {
    // Yukarı motorunu çalıştır
    digitalWrite(UP_MOTOR_PIN, HIGH);
    digitalWrite(DOWN_MOTOR_PIN, LOW);
}

void moveDown() {
    // Aşağı motorunu çalıştır
    digitalWrite(UP_MOTOR_PIN, LOW);
    digitalWrite(DOWN_MOTOR_PIN, HIGH);
}

void stopMotors() {
    // Tüm motorları durdur
    digitalWrite(FRONT_LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(FRONT_RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(FRONT_LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(FRONT_RIGHT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(BACK_LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(BACK_RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(BACK_LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(BACK_RIGHT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(UP_MOTOR_PIN, LOW);
    digitalWrite(DOWN_MOTOR_PIN, LOW);
}
