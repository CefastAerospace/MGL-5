#ifndef LUXLIST_H
#define LUXLIST_H

#include <Arduino.h>
#include <LinkedList.h>
#include <LuxItem.h>

// Classe que representa um mapa de iluminância por ângulo
class LuxList {
    public:
        // Métodos públicos
        LuxList();  // Construtor
        bool addLuxItem(LuxItem luxItem);
        LuxItem getLuxItem(int angulo);

    private:
        // Atributos privados
        LinkedList<LuxItem> luxList;
};

#endif