#include <stdio.h>
#include <ctype.h>

char *alphabeticalMorse[] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---",
    "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-",
    "..-", "...-", ".--", "-..-", "-.--", "--.."};
char *numericalMorse[] = {
    "-----", ".----", "..---", "...--", "....-",
    ".....", "-....", "--...", "---..", "----."};

/*
Find the index of an array in an array of char array 
*/
int findIndexByElement(char **arr, int size, char *elem)
{
    // for debugging purposes
    char *b = elem;
    for (int i = 0; i < size; i++)
    {
        char *a = arr[i];
        if (strcmp(a, b) == 0)
        {
            return i;
        }
    }
    return -1; // element does not exist
}

/*
Converts a morse code to text
char **mArr: Array of char array. Each char array contains a separate morse code
char *buff: Output array passed by ref. 
int size: size of the morse code array (mArr)

Note: Ideally, buff and mArr have the same size.
*/
void convertMorseToText(char **mArr, char *buff, int size)
{
    for (int i = 0; i < size; i++)
    {
        int idx = findIndexByElement(alphabeticalMorse, 26, mArr[i]);
        if (idx == -1) {
            int idx = findIndexByElement(numericalMorse, 10, mArr[i]);
            buff[i] = '0' + idx;
        } else{
            buff[i] = 'a' + idx;
        }
    }
}

/*
Converts a text to morse code
char *cArr: String to convert.
char **buff: Output array passed by ref. It is an array of character arrays.
int size: size of text to concer (mArr)

Note: Ideally, buff and mArr have the same size.
*/
void convertTextToMorse(char *cArr, char **buff, int size)
{

    for (int i = 0; i < size; i++)
    {
        char c = cArr[i];
        char *morse;
        c = tolower(c);
        if (islower(c))
        {
            morse = alphabeticalMorse[c - 'a'];
        }
        else if (isdigit(c))
        {
            morse = numericalMorse[c - '0'];
        }
        else if (isspace(c))
        {
            morse = ' ';
        }
        else
        {
            morse = ' ';
        }

        buff[i] = morse;
    }
}

int main(void)
{
    printf("Morse code conversion program: \n");

    /*Translate text to morse code*/
    char textArr[5] = {'H', 'E', 'L', 'L', 'O'};
    int textArrSize = (sizeof textArr / sizeof textArr[0]);
    char *buffer[textArrSize];
    convertTextToMorse(textArr, buffer, textArrSize);
    printf("\n\n\nText: %s", textArr);
    printf("\nText to Morse: ");
    for (int i = 0; i < 5; i++)
        printf("%s ", buffer[i]);

    /*Translate morse code to text*/
    char *morseArr[5] = {"....", ".", ".-..", ".-..", "---"};
    int morseArrSize = (sizeof morseArr / sizeof morseArr[0]);
    char buffer2[morseArrSize];
    convertMorseToText(morseArr, buffer2, morseArrSize);
    printf("\n\n\nMorse: ");
    for (int i = 0; i < morseArrSize; i++)
        printf("%s ", morseArr[i]);


    printf("\nMorse to Text: ");
    printf("%s ", buffer2);

    /*Fin*/
    printf("\n\n\ndone!s");

    return 0;
}