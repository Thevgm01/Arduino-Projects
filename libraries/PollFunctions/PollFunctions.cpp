#include <PollFunctions.h>

static unsigned long curMillis = 0;

Polls::Polls(unsigned long updateDelay) {
    startMillis = random(updateDelay);
    lengthMillis = updateDelay;
}

bool Polls::update() {
    if (!checkPoll()) return false;
    resetPoll();
    return true;
}

void Polls::setMillis(unsigned long millis) {
    curMillis = millis;
}

unsigned long Polls::getLengthMillis() {
    return lengthMillis;
}

unsigned long Polls::getPollElapsedMillis() {
    return curMillis - startMillis;
}

unsigned long Polls::getPollRemainingMillis() {
    return lengthMillis - getPollElapsedMillis();
}

bool Polls::checkPoll() {
    return lengthMillis > 0 && curMillis - startMillis >= lengthMillis || 
           lengthMillis == 1; // Run every frame
}

void Polls::resetPoll() {
    startMillis = curMillis;
}

bool Polls::isActive() {
    return lengthMillis > 0;
}

void Polls::setPollLength(const unsigned long millis) {
    lengthMillis = millis;
}