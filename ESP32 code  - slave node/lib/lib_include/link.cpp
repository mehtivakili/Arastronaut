#include "link.h"   // Include the link header file
#include <Arduino.h> // Include Arduino base library

MyLink* init_link() {
    MyLink* new_link = new MyLink(); // Create a new MyLink object
    new_link->next = NULL; // Set the next pointer to NULL
    return new_link; // Return the new link
}

void add_link(MyLink *p, uint16_t addr) {
    MyLink* new_link = new MyLink(); // Create a new MyLink object
    new_link->anchor_addr = addr; // Set the anchor address
    new_link->next = NULL; // Set the next pointer to NULL
    while (p->next != NULL) {
        p = p->next; // Traverse to the end of the link
    }
    p->next = new_link; // Add the new link to the end
}

MyLink* find_link(MyLink *p, uint16_t addr) {
    while (p != NULL) {
        if (p->anchor_addr == addr) { // Check if the address matches
            return p; // Return the matching link
        }
        p = p->next; // Traverse to the next link
    }
    return NULL; // Return NULL if no match is found
}

void update_link(MyLink *p, uint16_t addr, float range, float dbm) {
    MyLink* link = find_link(p, addr); // Find the link with the given address
    if (link != NULL) {
        link->range[0] = range; // Update the range
        link->dbm = dbm; // Update the dbm value
    }
}

void print_link(MyLink *p) {
    while (p != NULL) {
        Serial.print("Anchor: "); // Print anchor label
        Serial.print(p->anchor_addr); // Print anchor address
        Serial.print(" Range: "); // Print range label
        Serial.println(p->range[0]); // Print range value
        p = p->next; // Traverse to the next link
    }
}

void delete_link(MyLink *p, uint16_t addr) {
    MyLink* prev = NULL; // Pointer to the previous link
    while (p != NULL) {
        if (p->anchor_addr == addr) { // Check if the address matches
            if (prev != NULL) {
                prev->next = p->next; // Remove the link from the chain
            }
            delete p; // Delete the link
            return;
        }
        prev = p; // Move to the next link
        p = p->next; // Traverse to the next link
    }
}
