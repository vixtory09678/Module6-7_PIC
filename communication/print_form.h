#ifndef PRINT_FORM
#define PRINT_FORM

void printFloat(float n)
{
    int intpart = (int)n;
    int decpart = (n - intpart) * 1000;
    decpart = abs(decpart);
    if (n < 0.0f)
        printf("-%d.%d%d%d", abs(intpart), (decpart < 100) ? 0 : decpart / 100, (decpart < 10) ? 0 : (decpart / 10) % 10, decpart % 10);
    else
        printf("%d.%d%d%d", intpart, (decpart < 100) ? 0 : decpart / 100, (decpart < 10) ? 0 : (decpart / 10) % 10, decpart % 10);
}

// void printFloat(float n)
// {
//     char string[20];
//     sprintf(string, "\f%6.3f", n);
//     printf("%s", string);
// }

#endif