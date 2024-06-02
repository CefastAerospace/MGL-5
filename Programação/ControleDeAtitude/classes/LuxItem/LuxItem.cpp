#include "LuxItem.h"

// Construtor
LuxItem::LuxItem(int angulo, int iluminancia) {
    this->angulo = angulo;
    this->iluminancia = iluminancia;
}

// Construtor vazio 
LuxItem::LuxItem() {
    this->angulo = -1;
    this->iluminancia = 0;
}

