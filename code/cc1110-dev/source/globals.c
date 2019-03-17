#include "../include/globals.h"

void ERROR (char log[2][16])
{
	do
	{
		halBuiLcdUpdate(log[0], log[1]);
	} while (halBuiButtonPushed());
}