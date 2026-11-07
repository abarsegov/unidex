#pragma once

bool cardReaderAvailable();
// Call only with no background card users. Unmounts, then requests a one-shot reader boot.
void cardReaderRequest();
// After powerInit, before Serial/launcher/storage initialization. True owns this entire boot.
bool cardReaderBegin();
void cardReaderPoll();
