
# TESTING speak_out_loud

*README is available on languages: [English](README.md), [Russian](README.ru.md)*

*[TESTING scenarios](TESTS.md)*

## Testing priorites in multi-client regime
For testing open four terminal session.
In first one (TS1) start main launch file, on TS2, TS3 launch client emulator.
```shell 
#TS1
roslaunch speak_out_loud speak_out_loud.launch language:=en
#TS2
roslaunch speak_out_loud speak_out_loud_client_example.launch node_name:=client1
#TS3
roslaunch speak_out_loud speak_out_loud_client_example.launch node_name:=client2
```
In scenarios fragments of "The Fellowship of the Ring" by J.R.R.Tolkien are used.
There's nothing magical or special in them. Feel free to use any text you like.

### Scenario 1
IMPORTANT subqueue
```shell
#TS2
Text to say: Bilbo was very rich and very peculiar 
Priority [1-5]: 1
Text to say: Bilbo was very rich and very peculiar 
Priority [1-5]: 1
```
> ___Result:___ every text is read with male voice one after another

### Scenario 2
MESSAGE subqueue
```shell
#TS2
Text to say: Bilbo was very rich and very peculiar 
Priority [1-5]: 2
Text to say: Bilbo was very rich and very peculiar 
Priority [1-5]: 2
```
>___Result:___ every text is read with female voice one after another

### Scenario 3
TEXT messages
```shell
#TS2
Text to say: Bilbo was very rich and very peculiar 
Priority [1-5]: 3
Text to say: Bilbo was very rich and very peculiar 
Priority [1-5]: 3
```
>___Result:___ Texts are read with female voice, every new text cancels a reading one. 

### Scenario 4
NOTIFICATION messages
```shell
#TS2
Text to say: Bilbo was very rich and very peculiar 
Priority [1-5]: 4
Text to say: Bilbo was very rich and very peculiar 
Priority [1-5]: 4
```
>___Result:___ Texts are read with female voice, every new text cancels a reading one. 

### Scenario 5
PROGRESS messages
```shell
#TS2
Text to say: When Mr. Bilbo Baggins of Bag End announced that he would shortly be celebrating his eleventy-first birthday with a party of special magnificence,
Priority [1-5]: 5
Text to say:  Bilbo was very rich and very peculiar
Priority [1-5]: 5
```
>___Result:___ every text is read with female voice one after another

### Scenario 6
IMPORTANT + other types
```shell
#TS2
Text to say: When Mr. Bilbo Baggins of Bag End announced that he would shortly be celebrating his eleventy-first birthday with a party of special magnificence, there was much talk and excitement in Hobbiton.
Priority [1-5]: 1
Text to say: 2 
Priority [1-5]: 2
Text to say: 3 
Priority [1-5]: 3
Text to say: 4 
Priority [1-5]: 4
```
> ___Result:___ first text is read with male voice, none of others would be read, while text is read;
> Following texts are read with female voice in order: MESSAGE, TEXT;
> NOTIFICATION is canceled - there're texts in queue.

### Scenario 7
MESSAGE + other types
```shell
#TS2
Text to say: When Mr. Bilbo Baggins of Bag End announced that he would shortly be celebrating his eleventy-first birthday with a party of special magnificence, there was much talk and excitement in Hobbiton. Bilbo was very rich and very peculiar, and had been the wonder of the Shire for sixty years, ever since his remarkable disappearance and unexpected return. 
Priority [1-5]: 2
Text to say: 2 
Priority [1-5]: 2
Text to say: 3 
Priority [1-5]: 3
Text to say: 4
Priority [1-5]: 4
Text to say: 1
Priority [1-5]: 1
```
> ___Result:___ first text is read with female voice, 
> IMPORTANT interrupts reading and set as current read text with male voice, 
> Following texts are read with female voice in order: MESSAGE, TEXT;
> NOTIFICATION is canceled - there're texts in queue.

### Scenario 8
TEXT + other types
```shell
#TS2
Text to say: When Mr. Bilbo Baggins of Bag End announced that he would shortly be celebrating his eleventy-first birthday with a party of special magnificence, there was much talk and excitement in Hobbiton. Bilbo was very rich and very peculiar, and had been the wonder of the Shire for sixty years, ever since his remarkable disappearance and unexpected return. 
Priority [1-5]: 3
Text to say: 4
Priority [1-5]: 4
Text to say: 2 
Priority [1-5]: 2
Text to say: 1
Priority [1-5]: 1
```
> ___Result:___ first text is read with female voice, 
> MESSAGE interrupts reading and set as current read text with female voice, 
> IMPORTANT interrupts reading and set as current read text with male voice, 
> NOTIFICATION is canceled - there's a reading text.

### Scenario 9
NOTIFICATION + other types
```shell
#TS2
Text to say: When Mr. Bilbo Baggins of Bag End announced that he would shortly be celebrating his eleventy-first birthday with a party of special magnificence, there was much talk and excitement in Hobbiton. Bilbo was very rich and very peculiar, and had been the wonder of the Shire for sixty years, ever since his remarkable disappearance and unexpected return. 
Priority [1-5]: 4
Text to say: 4
Priority [1-5]: 4
Text to say: 3
Priority [1-5]: 3
Text to say: 2 
Priority [1-5]: 2
Text to say: 1
Priority [1-5]: 1
```
> ___Result:___ first text is read with female voice, 
> NOTIFICATION interrupts reading and set as current read text with female voice, 
> TEXT interrupts reading and set as current read text with female voice, 
> MESSAGE interrupts reading and set as current read text with female voice, 
> IMPORTANT interrupts reading and set as current read text with male voice, 

### Scenario 10
Whitelisting/Blacklisting, turning them on/off

#### Whitelisting
```shell
#TS2
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 

#TS3
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 
```
> ___Result:___ both texts are read 

```shell
#TS4
rostopic pub /speak_out_loud_whitelist std_msgs/String "data: '/client1'" -1

#TS2
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 

#TS3
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 
```
> ___Result:___ only text from TS2 is read 

```shell

#TS4
rostopic pub /speak_out_loud_whitelist std_msgs/String "data: '/client2'" -1

#TS2
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 

#TS3
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 

```
> ___Result:___ both fragments are read

#### Turn blacklist on

```shell
#TS4
rostopic pub /speak_out_loud_blacklist_on std_msgs/Bool "data: true" -1
```
> When turning whilelist or blacklist on, another one is turned off

#### Blacklisting

```shell
#TS2
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 

#TS3
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 
```
> ___Result:___ both fragments are read

```shell
#TS4
rostopic pub /speak_out_loud_blacklist std_msgs/String "data: '/client1'" -1

#TS2
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 

#TS3
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 
```
> ___Result:___ text from TS2 is not read.

```shell
#TS4
rostopic pub /speak_out_loud_blacklist std_msgs/String "data: '/client2'" -1

#TS2
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 

#TS3
Text to say: When Bilbo was ninety-nine
Priority [1-5]: 
```
> ___Result:___ silence

#### Turn blacklist off
```shell
#TS4
rostopic pub /speak_out_loud_blacklist_on std_msgs/Bool "data: false" -1
```
> ___Result:___ blacklist is off

### Scenario 11
Debugging

Restart TS3 in debug mode:
```shell 
#TS3
roslaunch speak_out_loud speak_out_loud_client_example.launch node_name:=client2 debug:=True
```
Send some text
```shell
#TS2
Text to say: 1
Priority [1-5]: 
#TS3
Text to say: 2
Priority [1-5]: 
```
>___Result:___ only text from TS2 is read

Restart all terminals, use debug mode in TS1 and TS3:
```shell 
#TS1
roslaunch speak_out_loud speak_out_loud.launch language:=en debug:=True
#TS2
roslaunch speak_out_loud speak_out_loud_client_example.launch node_name:=client1
#TS3
roslaunch speak_out_loud speak_out_loud_client_example.launch node_name:=client2 debug:=True
```
Send some text
```shell
#TS2
Text to say: 1
Priority [1-5]: 
#TS3
Text to say: 2
Priority [1-5]: 
```
>___Result:___ both texts are read
