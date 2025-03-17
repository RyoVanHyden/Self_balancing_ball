#include "ServoHiwonderClass.hpp"

ServoController::ServoController(Stream &serialX) : serialX(serialX)
{
}

void ServoController::send(const uint8_t id, const uint8_t cmd, const uint8_t numberOfParam, uint8_t pram1...)
{
    serialX.flush();
    sendPack.clear();

    SERIAL_DEBUG(F("PUSHING"));
    SERIAL_DEBUG(SIGNATURE);
    SERIAL_DEBUG(SIGNATURE);

    sendPack.pushback(SIGNATURE);
    sendPack.pushback(SIGNATURE);

    uint8_t len = numberOfParam + 3;
    uint8_t checkSum = id + cmd + len;

    SERIAL_DEBUG(id);
    SERIAL_DEBUG(len);
    SERIAL_DEBUG(cmd);

    sendPack.pushback(id);
    sendPack.pushback(len);
    sendPack.pushback(cmd);

    if (numberOfParam != 0)
    {
        va_list vl;
        va_start(vl, pram1);
        SERIAL_DEBUG(pram1);
        sendPack.pushback(pram1);

        checkSum += pram1;
        for (uint8_t i = 1; i < numberOfParam; i++)
        {
            uint8_t val = static_cast<uint8_t>(va_arg(vl, int));
            checkSum += val;
            SERIAL_DEBUG(val);
            sendPack.pushback(val);
        }
        va_end(vl);
    }

    checkSum = ~checkSum;

    SERIAL_DEBUG(checkSum);
    sendPack.pushback(checkSum);

    SERIAL_DEBUG(F("SENDING PACK"));

    serialX.write(sendPack.buffer, sendPack.bufferSize);
}

uint8_t ServoController::getExpectedLen(const uint8_t cmd)
{
    switch (cmd)
    {
    case SERVO_MOVE_TIME_WRITE:
    case SERVO_MOVE_TIME_WAIT_WRITE:
    case SERVO_ANGLE_LIMIT_WRITE:
    case SERVO_VIN_LIMIT_WRITE:
    case SERVO_OR_MOTOR_MODE_WRITE:
        return 7;

    case SERVO_ID_WRITE:
    case SERVO_ANGLE_OFFSET_ADJUST:
    case SERVO_TEMP_MAX_LIMIT_WRITE:
    case SERVO_LOAD_OR_UNLOAD_WRITE:
    case SERVO_LED_CTRL_WRITE:
    case SERVO_LED_ERROR_WRITE:
        return 4;

    case SERVO_MOVE_TIME_READ:
    case SERVO_MOVE_TIME_WAIT_READ:
    case SERVO_MOVE_START:
    case SERVO_MOVE_STOP:
    case SERVO_ID_READ:
    case SERVO_ANGLE_OFFSET_WRITE:
    case SERVO_ANGLE_OFFSET_READ:
    case SERVO_ANGLE_LIMIT_READ:
    case SERVO_VIN_LIMIT_READ:
    case SERVO_TEMP_MAX_LIMIT_READ:
    case SERVO_TEMP_READ:
    case SERVO_VIN_READ:
    case SERVO_POS_READ:
    case SERVO_OR_MOTOR_MODE_READ:
    case SERVO_LOAD_OR_UNLOAD_READ:
    case SERVO_LED_CTRL_READ:
    case SERVO_LED_ERROR_READ:
        return 3;

    default:
        return 0; // Return 0 or another appropriate value for unrecognized commands
    }
}

bool ServoController::recv(const uint8_t id, const uint8_t cmd)
{
    // example of expected SERVO_MOVE_TIME_WRITE
    //  0x55 0x55 ID 7 SERVO_MOVE_TIME_WRITE   low   high low high checksum
    //   1     2   3  4              5          6     7    8  9     10
    //  7 + 3 => 10 = totalLen
    uint8_t len = getExpectedLen(cmd);
    uint8_t totalLen = len + 3;
    if (len == 0)
    {
        return false; // Return error if command length is unknown
    }

    unsigned long startTime = millis();
    while (Serial.available() < totalLen)
    {
        if (millis() - startTime > TIMEOUT)
        {
            SERIAL_DEBUG(F("TIMEOUT"));
            return false; // Timeout if data is not available
        }
    }

    // Read the packet into the buffer    1     2   3    |      (numberofParam + 3)           |
    uint8_t recvBuffer[totalLen]; // 0x55 0x55  id   len   cmd param1 param2 ...  checksum       yeah I forgot that len only counts from len forward it needs a + 3 for the SIGNATURE SIGNATURE ID
    Serial.readBytes(recvBuffer, totalLen);

    // Validate header
    if (recvBuffer[0] != SIGNATURE || recvBuffer[1] != SIGNATURE)
    {
        SERIAL_DEBUG(F("SIGNATURE IS WRONG"));
        return false; // Return error if header is incorrect
    }

    // Validate ID
    if (recvBuffer[2] != id)
    {
        SERIAL_DEBUG(F("ID IS WRONG"));
        return false; // Return error if ID does not match
    }

    // Validate Length
    if (recvBuffer[3] != len)
    { // Length should equal the total length of the packet
        SERIAL_DEBUG(F("LEN IS WRONG"));
        return false; // Return error if length does not match
    }

    // Validate Command
    if (recvBuffer[4] != cmd)
    {
        SERIAL_DEBUG(F("CMD IS WRONG"));
        return false; // Return error if command does not match
    }

    // Calculate and validate checksum
    uint8_t checksum = 0;
    for (int i = 2; i < totalLen - 1; ++i)
    { // Start from ID to last parameter
        checksum += recvBuffer[i];
    }
    checksum = ~checksum;

    if (recvBuffer[totalLen - 1] != checksum)
    {
        SERIAL_DEBUG(F("CHECKSUM IS WRONG"));
        return false; // Return error if checksum does not match
    }

    SERIAL_DEBUG(F("SUCESS"));
    recvPack.setPacket(recvBuffer, totalLen);

    return true; // Success
}

Packet ServoController::getPacket() const
{
    return recvPack;
}

void ServoController::setID(const uint8_t oldID, const uint8_t newID)
{
    send(oldID, SERVO_ID_WRITE, 1, newID);
}

bool ServoController::getID(uint8_t *recvID)
{
    if (!recvID)
    {
        SERIAL_DEBUG(F("NULL POINTER"));
        return false;
    }

    send(ALL_SERVOS, SERVO_ID_READ, 0, 0);

    if (recv(ALL_SERVOS, SERVO_ID_READ))
    {
        *recvID = recvPack.buffer[5];
        SERIAL_DEBUG(F("RECEIVED: "));
        SERIAL_DEBUG(*recvID);
        return true;
    }
    else
    {
        SERIAL_DEBUG(F("getID FAILED"));
        return false;
    }
}

void ServoController::moveWithTime(const uint8_t id, int16_t position, uint16_t time)
{
    position = constrain(position, 0, 1000);
    time = constrain(time, 0, 30000);
    uint8_t param1 = lowByte(position);
    uint8_t param2 = highByte(position);
    uint8_t param3 = lowByte(time);
    uint8_t param4 = highByte(time);
    send(id, SERVO_MOVE_TIME_WRITE, 4, param1, param2, param3, param4);
}

void ServoController::moveWithTime(ServoHiwonder &servo)
{
    moveWithTime(servo.id, servo.position, servo.time);
}

void ServoController::moveAll(int16_t position, uint16_t time)
{
    moveWithTime(ALL_SERVOS, position, time);
}

bool ServoController::getPosWithTime(const uint8_t id, int16_t *pos, uint16_t *time)
{

    send(id, SERVO_POS_READ, 0, 0);
    if (recv(id, SERVO_POS_READ))
    {

        //  0    1    2  3   4    5      6      7      8       9
        // 0x55 0x55 id cmd len param1 param2 param3 param4 checkSum
        //                      low(Pos)high(Pos)low(Time)high(Time)
        if (pos != nullptr)
        {
            *pos = recvPack.combine(6, 5);
            *pos = constrain(*pos, 0, 1000);
        }
        else
        {
            SERIAL_DEBUG(F("POS IS NULLPTR -> NOT FETCHING POS"));
        }

        if (time != nullptr)
        {
            *time = recvPack.combine(8, 7);
        }
        else
        {
            SERIAL_DEBUG(F("TIME IS NULLPTR -> NOT FETCHING TIME"));
        }

        return true;
    }
    else
    {
        SERIAL_DEBUG(F("getPosWithTime FAILED"));
        return false;
    }
}

bool ServoController::getPosWithTime(ServoHiwonder &servo)
{
    return getPosWithTime(servo.id, &(servo.position), &(servo.time));
}

void ServoController::waitFor(const uint16_t time)
{
    unsigned long prev = millis();
    while (millis() - prev <= time)
    {
        delay(1);
    }
}

void ServoController::storeMWT_WaitForSignal(const uint8_t id, int16_t position, uint16_t time)
{
    position = constrain(position, 0, 1000);
    time = constrain(time, 0, 30000);
    uint8_t param1 = lowByte(position);
    uint8_t param2 = highByte(position);
    uint8_t param3 = lowByte(time);
    uint8_t param4 = highByte(time);

    send(id, SERVO_MOVE_TIME_WAIT_WRITE, 4, param1, param2, param3, param4);
}

void ServoController::storeMWT_WaitForSignal(ServoHiwonder &servo)
{
    storeMWT_WaitForSignal(servo.id, servo.position, servo.time);
}

void ServoController::sendSignal(const uint8_t id)
{
    send(id, SERVO_MOVE_START, 0, 0);
}

bool ServoController::getPosFromWFS(const uint8_t id, int16_t *pos)
{
    if (!pos)
    {
        SERIAL_DEBUG(F("POS IS NULLPTR -> CANNOT FETCH POSITION"));
        return false;
    }

    send(id, SERVO_MOVE_TIME_WAIT_READ, 0, 0);

    if (recv(id, SERVO_MOVE_TIME_WAIT_READ))
    {
        *pos = recvPack.combine(6, 5);
        *pos = constrain(*pos, 0, 1000);

        SERIAL_DEBUG(F("getPosFromWFS SUCCESS"));
        return true;
    }
    else
    {
        SERIAL_DEBUG(F("getPosFromWFS FAILED"));
        return false;
    }
}

bool ServoController::getPosFromWFS(ServoHiwonder &servo)
{
    return getPosFromWFS(servo.id, &(servo.position));
}

void ServoController::stopMoving(const uint8_t id)
{
    send(id, SERVO_MOVE_STOP, 0, 0);
}

void ServoController::stopMoving(ServoHiwonder &servo)
{
    stopMoving(servo.id);
}

void ServoController::adjustOffset(const uint8_t id, int8_t adjust)
{
    send(id, SERVO_ANGLE_OFFSET_ADJUST, 1, adjust);
}

void ServoController::adjustOffset(ServoHiwonder &servo, int8_t adjust)
{
    adjustOffset(servo.id, adjust);
}

void ServoController::writeOffset(const uint8_t id)
{
    send(id, SERVO_ANGLE_LIMIT_WRITE, 0, 0);
}

void ServoController::writeOffset(ServoHiwonder &servo)
{
    writeOffset(servo.id);
}

bool ServoController::getOffset(const uint8_t id, int8_t *offset)
{
    if (!offset)
    {
        SERIAL_DEBUG(F("OFFSET IS NULLPTR -> CANNOT FETCH OFFSET"));
        return false;
    }

    send(id, SERVO_ANGLE_OFFSET_READ, 0, 0);
    if (recv(id, SERVO_ANGLE_OFFSET_READ))
    {
        *offset = static_cast<int8_t>(recvPack.buffer[5]);
        SERIAL_DEBUG(F("getOffset SUCCESS"));
        return true;
    }
    else
    {
        SERIAL_DEBUG(F("getOffset FAILED"));
        return false;
    }
}

bool ServoController::getOffset(ServoHiwonder &servo, int8_t *offset)
{
    return getOffset(servo.id, offset);
}

void ServoController::setLimits(const uint8_t id, uint8_t min, uint8_t max)
{
    min = constrain(min, 0, 1000);
    max = constrain(max, 0, 1000);

    if (min == max)
    {
        return;
    }

    if (min > max)
    {
        uint8_t aux = min;
        min = max;
        max = aux;
    }

    uint8_t param1 = lowByte(min);
    uint8_t param2 = highByte(min);
    uint8_t param3 = lowByte(max);
    uint8_t param4 = highByte(max);

    send(id, SERVO_ANGLE_LIMIT_WRITE, 4, param1, param2, param3, param4);
}

void ServoController::setLimits(ServoHiwonder &servo, uint8_t min, uint8_t max)
{
    setLimits(servo.id, min, max);
}

bool ServoController::getPos(const uint8_t id, int16_t *pos)
{
    if (!pos)
    {
        SERIAL_DEBUG(F("POS IS NULLPTR -> CANNOT FETCH POSITION"));
        return false;
    }

    send(id, SERVO_POS_READ, 0, 0);

    if (recv(id, SERVO_POS_READ))
    {
        *pos = recvPack.combine(6, 5);
        *pos = constrain(*pos, 0, 1000);

        SERIAL_DEBUG(F("getPos SUCCESS"));
        return true;
    }
    else
    {
        SERIAL_DEBUG(F("getPos FAILED"));
        return false;
    }
}

bool ServoController::getTemp(const uint8_t id, uint8_t *temp)
{
    if (!temp)
    {
        SERIAL_DEBUG(F("TEMP IS NULLPTR -> CANNOT FETCH TEMPERATURE"));
        return false;
    }

    send(id, SERVO_TEMP_READ, 0, 0);

    if (recv(id, SERVO_TEMP_READ))
    {
        *temp = recvPack.buffer[5];
        SERIAL_DEBUG(F("getTemp SUCCESS"));
        return true;
    }
    else
    {
        SERIAL_DEBUG(F("getTemp FAILED"));
        return false;
    }
}

bool ServoController::getVin(const uint8_t id, uint16_t *vin)
{
    if (!vin)
    {
        SERIAL_DEBUG(F("VIN IS NULLPTR -> CANNOT FETCH TEMPERATURE"));
        return false;
    }

    send(id, SERVO_TEMP_READ, 0, 0);
    if (recv(id, SERVO_TEMP_READ))
    {
        *vin = recvPack.combine(6, 5);
        SERIAL_DEBUG(F("getVin SUCCESS"));
        return true;
    }
    else
    {
        SERIAL_DEBUG(F("getVin FAILED"));
        return false;
    }
}

void ServoController::turnLedOn(const uint8_t id)
{
    send(id, SERVO_LED_CTRL_WRITE, 1, static_cast<uint8_t>(0));
}
void ServoController::turnLedOff(const uint8_t id)
{
    send(id, SERVO_LED_CTRL_WRITE, 1, static_cast<uint8_t>(1));
}

void ServoController::turnLed(const uint8_t id, const bool on)
{
    on ? turnLedOn(id) : turnLedOff(id);
}

bool ServoController::getLed(const uint8_t id, bool *isOn)
{
    if (!isOn)
    {
        SERIAL_DEBUG(F("ISON IS NULLPTR -> CANNOT FETCH ISON"));
        return false;
    }

    send(id, SERVO_LED_CTRL_READ, 0, 0);
    if (recv(id, SERVO_LED_CTRL_READ))
    {
        if (recvPack.buffer[5] == static_cast<uint8_t>(1))
        {
            *isOn = false;
        }
        else
        {
            *isOn = true;
        }
        SERIAL_DEBUG(F("getLed SUCCESS"));
        return true;
    }
    else
    {
        SERIAL_DEBUG(F("getLed FAILED"));
        return false;
    }
}

void ServoController::changeLed(const uint8_t id)
{
    bool isOn = false;
    if (getLed(id, &isOn))
    {
        if (isOn)
        {
            turnLedOff(id);
        }
        else
        {
            turnLedOff(id);
        }
    }
}

void ServoController::turnLedOn(const ServoHiwonder &servo)
{
    turnLedOn(servo.id);
}

void ServoController::turnLedOff(const ServoHiwonder &servo)
{
    turnLedOff(servo.id);
}

void ServoController::turnLed(const ServoHiwonder &servo, const bool on)
{
    turnLed(servo.id, on);
}

bool ServoController::getLed(ServoHiwonder &servo)
{
    return getLed(servo.id, &(servo.isLedOn));
}

void ServoController::changeLed(const ServoHiwonder &servo)
{
    changeLed(servo.id);
}

void ServoController::setLoadMode(const uint8_t id)
{
    send(id, SERVO_LOAD_OR_UNLOAD_WRITE, 1, 1);
}

void ServoController::setLoadMode(const ServoHiwonder &servo)
{
    setLoadMode(servo.id);
}

void ServoController::setUnloadMode(const uint8_t id)
{
    send(id, SERVO_LOAD_OR_UNLOAD_WRITE, 1, 0);
}

void ServoController::setUnloadMode(const ServoHiwonder &servo)
{
    setUnloadMode(servo.id);
}

bool ServoController::getLoadOrUnload(const uint8_t id, bool *isLoadMode)
{

    if (!isLoadMode)
    {
        SERIAL_DEBUG(F("ISLOADMODE IS NULLPTR -> CANNOT FETCH ISLOADMODE"));
        return false;
    }

    send(id, SERVO_LOAD_OR_UNLOAD_READ, 0, 0);
    if (recv(id, SERVO_LOAD_OR_UNLOAD_READ))
    {
        switch (recvPack.buffer[5])
        {
        case 1:
            *isLoadMode = true;
            break;
        case 0:
            *isLoadMode = false;
            break;
        default:
            return false;
            break;
        }
        SERIAL_DEBUG(F("getLoadOrUnload SUCCESS"));

        return true;
    }
    else
    {
        SERIAL_DEBUG(F("getLoadOrUnload FAILED"));
        return false;
    }
}

bool ServoController::getLoadOrUnload(ServoHiwonder &servo)
{
    return getLoadOrUnload(servo.id, &(servo.isLoadMode));
}

void ServoController::moveRelative(const uint8_t id, int16_t relative, uint16_t time)
{
    int16_t current = 0;
    if (!getPos(id, &current))
    {
        return;
    }

    int16_t target = relative + target;
    moveWithTime(id, target, time);
}

void ServoController::moveRelative(ServoHiwonder &servo, int16_t relative)
{
    int16_t current = 0;
    if (!getPos(servo.id, &current))
    {
        return;
    }

    servo.position = current + relative;
    moveWithTime(servo);
}

float easeInSine(float x)
{
    return 1 - cos((x * M_PI) / 2);
}

float easeOutSine(float x)
{
    return sin((x * M_PI) / 2);
}

float easeInOutSine(float x)
{
    return -(cos(M_PI * x) - 1) / 2;
}

float easeInCubic(float x)
{
    return x * x * x;
}

float easeOutCubic(float x)
{
    return 1 - pow(1 - x, 3);
}

float easeInOutCubic(float x)
{
    return (x < 0.5) ? (4 * x * x * x) : (1 - pow(-2 * x + 2, 3) / 2);
}

float gaussian(float x, float b = 0.5, float c = 0.1)
{
    return exp(-0.5 * pow((x - b) / c, 2.0));
}

void ServoController::moveAnim(const uint8_t id, int16_t pos, uint16_t totalTime, anim animType)
{
    pos = constrain(pos, 0, 1000);
    int16_t currentPos = 0;
    uint16_t animTime = 1000;

    if (!getPos(id, &currentPos))
    {
        return;
    }
    int16_t dif = pos - currentPos;

    unsigned long startTime = millis();
    unsigned long currentTime = startTime;
    unsigned long endTime = startTime + totalTime;
    while (currentTime < endTime)
    {
        float elapsedTime = (currentTime - startTime) / static_cast<float>(totalTime);
        float easeFactor;

        // Switch case to choose the animation type
        switch (animType)
        {
        case anim::easeInSine:
            easeFactor = easeInSine(elapsedTime);
            break;
        case anim::easeOutSine:
            easeFactor = easeOutSine(elapsedTime);
            break;
        case anim::easeInOutSine:
            easeFactor = easeInOutSine(elapsedTime);
            break;
        case anim::easeInCubic:
            easeFactor = easeInCubic(elapsedTime);
            break;
        case anim::easeOutCubic:
            easeFactor = easeOutCubic(elapsedTime);
            break;
        case anim::easeInOutCubic:
            easeFactor = easeInOutCubic(elapsedTime);
            break;
        case anim::gaussian:
            easeFactor = gaussian(elapsedTime);
            break;
        default:
            easeFactor = elapsedTime; // Default to linear if unknown anim type
            break;
        }
        int16_t newPos = currentPos + static_cast<int16_t>(dif * easeFactor);
        newPos = constrain(newPos, 0, 1000);
        uint16_t remainingTime = endTime - currentTime;
        moveWithTime(id, newPos, remainingTime);
        currentTime = millis();
        waitFor(10);
    }
    moveWithTime(id, pos, 100);
}

void ServoController::moveAnim(ServoHiwonder &servo, anim animType)
{
    moveAnim(servo.id, servo.position, servo.time, animType);
}

void ServoController::moveMin(const uint8_t id, uint16_t totalTime, anim animType)
{
    if (animType == anim::none)
    {
        moveWithTime(id, 0, totalTime);
        return;
    }
    moveAnim(id, 0, totalTime, animType);
}

void ServoController::moveMin(ServoHiwonder &servo, anim animType)
{
    servo.position = 0;
    moveMin(servo.id, servo.time, animType);
}

void ServoController::moveMax(const uint8_t id, uint16_t totalTime, anim animType)
{
    if (animType == anim::none)
    {
        moveWithTime(id, 1000, totalTime);
        return;
    }
    moveAnim(id, 1000, totalTime, animType);
}

void ServoController::moveMax(ServoHiwonder &servo, anim animType)
{
    servo.position = 1000;
    moveMin(servo.id, servo.time, animType);
}

void ServoController::sequence(uint8_t *ids, uint8_t numberOfIds, int16_t **positions, uint8_t numberOfPositions, uint8_t numberOfCycles, uint16_t timePerMovement, uint16_t pause)
{
    for (uint8_t cycle = 0; cycle < numberOfCycles; ++cycle)
    {
        for (uint8_t pos = 0; pos < numberOfPositions; ++pos)
        {
            for (uint8_t id = 0; id < numberOfIds; ++id)
            {
                moveWithTime(ids[id], positions[id][pos], timePerMovement);
                waitFor(10); // so that i dont send a blob of data to the servos with the all the spam
            }
            waitFor(pause);
        }
    }
}

void ServoController::sequence(action &act, uint8_t numberOfCycles, uint16_t timePerMovement, uint16_t pause)
{
    for (uint8_t cycle = 0; cycle < numberOfCycles; ++cycle)
    {
        for (uint8_t pos = 0; pos < act.getMatrixSize(); pos++)
        {
            for (uint8_t id = 0; id < act.idsSize; id++)
            {
                moveWithTime(act.ids[id], act.positions[id][pos], timePerMovement);
                waitFor(10);
            }
            waitFor(pause);
        }
    }
}

void ServoController::domino(uint8_t *ids, uint8_t numberOfIds, int16_t *positions, uint8_t numberOfPositions, uint16_t timePerMovement, uint16_t pause, anim animType)
{
    for (uint8_t pos = 0; pos < numberOfPositions; pos++)
    {
        for (uint8_t id = 0; id < numberOfIds; id++)
        {
            moveAnim(ids[id], positions[pos], timePerMovement, animType);
        }
        waitFor(pause);
    }
}

bool ServoController::moveArm(uint8_t idBase, uint8_t idArm1, uint8_t idArm2, kine::points pts, uint16_t time, float len1, float len2)
{
    float o1 = 0;
    float o2 = 0;
    float o3 = 0;

    if (pts.getOrientations(&o1, &o2, &o3, len1, len2))
    {
        if (o1 > 120 || o3 < -120)
        {
            Serial.println("Impossivel");
            return false;
        }
        float O1 = map(o1, -120, 120, 0, 1000);
        float O2 = map(o2, 0, 180, 0, 1000);
        float O3 = map(o3, -90, 90, 0, 1000);

        moveWithTime(idBase, O1, time);
        waitFor(5);
        moveWithTime(idArm1, O2, time);
        waitFor(5);
        moveWithTime(idArm2, O3, time);
        waitFor(5);
        return true;
    }
    return false;
}
