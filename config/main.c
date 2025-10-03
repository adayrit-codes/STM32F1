// Minimal generic main used as a common startup for projects.
// It simply spins executing NOP instructions indefinitely.

int main(void)
{
	while (1) {
		__asm__ volatile ("nop");
	}
	return 0;
}