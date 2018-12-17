#include "cli.h"
#include "usart.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

// head of CLI command list
static CLI_CommandItem *head = NULL;

// char buffer where command will be stored
static char commandBuffer[100];
static char lowerCommandName[100];
static char lowerCommand[100];
static int  foundedCommandIndex;

/**
 * This function searches the CLI command list and tries to find a descriptor for the provided command.
 * The command format is case-insensitive.
 * Returns pointer to @ref CLI_MenuItem structure if given command was found in the CLI command list.
 *
 * @param command pointer to command (string)
 *
 * @retval pointer to @ref CLI_MenuItem structure desrcibing the command, if command was found
 * @retval NULL if the given command does not match any command regitstered in the CLI command list 
 */
static CLI_CommandItem* CLI_GetMenuItemByCommandName(char *command);

/**
 * @bref This function is responsible for collecting the command reading in from USART.
 *
 * This function should check wether the USART interface has some new data. If it does
 * this function reads new characters and stores them in a command buffer. This function
 * is also responsible for providing echo of the input characters back to the buffer.
 *
 * The function exits when:
 * - no more characters are available to read from USART - the function returns false
 * - new line was detected - this function returns true
 *
 * @retval true if command was collected
 * @retval false if command is yet incomplete
 */
static bool CLI_StoreCommand(void);

/**
 * @brief This function converts string to a lowercase
 *
 * @param dst pointer where converted null terminated string will be stored
 * @param src pointer to string which will be converted
 */
 
static void CLI_StringToLower(char *dst, const char *src);
	
	
	
void CLI_Proc(void){
	CLI_CommandItem *tempCmd;
	if(CLI_StoreCommand() == true){
		if(commandBuffer[0] == '?'){
			CLI_PrintAllCommands();
		}
		else{
			
			char backupBuffer[100];
			strcpy(backupBuffer,commandBuffer);
			
			tempCmd = CLI_GetMenuItemByCommandName(backupBuffer);
			
				if (tempCmd != NULL){
					
					char data[100];
					sprintf(data, "%s", backupBuffer + foundedCommandIndex +1);
					size_t dataLenght = strlen(data);
					while (dataLenght != 0){
						
						if(data[dataLenght] == '\n' || data[dataLenght] == '\r') {
							data[dataLenght] = '\0';
						}
						//printf("%i",dataLenght);   //printf zacinaja procesor gdy nie ustawione!
						dataLenght--;
						
					}
				//	printf("%s",data);
					
					tempCmd->callback(data);
				}
			}
		}
}

bool CLI_AddCommand(CLI_CommandItem *item){
        if ((item->callback != NULL) && (strlen(item->commandName) != 0)){
            if(head== NULL){
            head=item;
        }
        else {
            item->next=head;
            head =item;
        }
        return true;
    }
	return false;
}


void CLI_PrintAllCommands(void){
	
    CLI_CommandItem *temp = head;
    USART_WriteString("---------------------LISTA KOMEND-------------------- \n");
		int i=1;
    while(temp != NULL){
				USART_PutChar(i++);
        USART_WriteString("Nazwa polecenia: ");
        USART_WriteString(temp->commandName);
        USART_WriteString("\n");
        USART_WriteString("Opis:");
        USART_WriteString(temp->description);
        USART_WriteString("\n");
        temp=temp->next;
    }
}


CLI_CommandItem* CLI_GetMenuItemByCommandName(char *command){
	//todo
	CLI_CommandItem *temp = head;
			
	char* commandBufferTemp = command;
	CLI_StringToLower(lowerCommand,commandBufferTemp);
	size_t sizeTemp = strlen(lowerCommand);

	while(lowerCommand[0] != '\0'){
		while(temp != NULL){
			CLI_StringToLower(lowerCommandName,temp->commandName);
			if(strcmp(lowerCommandName, lowerCommand ) == 0){
				return temp;
			}
			else{
				temp = temp->next;
			}
		}
		lowerCommand[sizeTemp] = '\0';
		foundedCommandIndex = sizeTemp;
		sizeTemp--;
		temp = head;
	}
	return NULL;
};

void CLI_StringToLower(char *dst, const char *src){
	//todo proszę wykorzystać funkcje z biblioteki ctype.h
		char *dstTemp = dst;
		size_t length = strlen(src);
			while (length > 0){
				*dstTemp++ = tolower(*src++);
				length--;
			}
		*dstTemp=0;
}

bool CLI_StoreCommand(){
	
    static int i = 0;
     
    if(USART_GetChar(&commandBuffer[i])){
        if(commandBuffer[i] != '\n'){
           // USART_PutChar(commandBuffer[i]);   //echo
            if(i<99){
                i++;
            }
        }
        else{
            i=0;
            return true;
        }
    }
	
	return false;
}
