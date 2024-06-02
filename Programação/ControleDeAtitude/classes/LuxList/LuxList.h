#ifndef LUXLIST_H
#define LUXLIST_H

#include <Arduino.h>
#include <LinkedList.h>
#include <LuxItem.h>

// Classe que representa um mapa de iluminância por ângulo
class LuxList {
    public:
        LuxList();
        bool addLuxItem(LuxItem luxItem);
        LuxItem getLuxItem(int angulo);

    private:
        LinkedList<LuxItem> luxList;
};

#endif