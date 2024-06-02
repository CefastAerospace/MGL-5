#include "LuxList.h"

// Construtor
LuxList::LuxList() {
}

// Adiciona um item à lista
bool LuxList::addLuxItem(LuxItem luxItem) {
    for (int i = 0; i < luxList.size(); i++) {
        LuxItem tmp = luxList.get(i);
        if (tmp.angulo == luxItem.angulo) {
            // Modifica apenas a iluminância do ângulo já existente na lista
            tmp.iluminancia = luxItem.iluminancia;
            luxList.set(i, tmp); // Atualiza o item na lista
            return true;
        }
    }
    this->luxList.add(luxItem);
    return true;
}

// Retorna um item da lista
LuxItem LuxList::getLuxItem(int angulo) {
    for (int i = 0; i < luxList.size(); i++) {
        LuxItem tmp = luxList.get(i);
        if (tmp.angulo == angulo) {
            return tmp;
        }
    }
    return LuxItem(-1, 0);  // Retorna um item nulo
}