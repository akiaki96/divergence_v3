#pragma once

#define onselect(nodeName, func)  \
void nodeName##_onselect() { \
    func \
}

#define onenter(nodeName, func) \
void nodeName##_onenter() { \
    func \
}