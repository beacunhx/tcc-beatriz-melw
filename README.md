# Braço robótico separador de objetos por cor com saída em áudio

Protótipo de baixo custo de um braço robótico que identifica a cor de uma peça, anuncia a cor em voz alta e leva a peça até o nicho correspondente. A saída em áudio torna o sistema acessível a pessoas com deficiência na percepção cromática (daltonismo).

Projeto desenvolvido como Trabalho de Conclusão de Curso em **Engenharia de Controle e Automação** na **PUC-Campinas** (2026).

> **Monografia:** *Desenvolvimento de um braço robótico de baixo custo para identificação e separação de objetos por cor com saída acessível a pessoas com deficiência na percepção cromática*

---

## Funcionamento

1. O sensor TCS3200 lê a cor da peça a cada 2,5 s e mostra o resultado no Monitor Serial.
2. O operador autoriza o ciclo apertando o botão (ou enviando `s` pelo Monitor Serial).
3. **Cor identificada (vermelho, verde ou azul):** o módulo de áudio fala o nome da cor enquanto o braço pega a peça e a deposita no nicho daquela cor.
4. **Cor não identificada:** o sistema avisa por áudio e o braço permanece parado na posição inicial.

Os movimentos usam um perfil de velocidade em "S" (*smoothstep*), com todas as juntas chegando ao destino ao mesmo tempo, o que reduz trancos e vibração nos servos.

## Hardware

| Componente | Função |
|---|---|
| Arduino UNO | Controle |
| Kit de braço robótico MARK2 (4 servos) | Base, ombro, cotovelo e garra |
| Sensor de cor TCS3200 | Identificação da cor |
| DFPlayer Mini + cartão micro SD + alto-falante | Saída em áudio |
| Botão de pressão | Autorização do operador |
| Resistor de 1 kΩ | Proteção da linha RX do DFPlayer |

## Ligações

| Dispositivo | Pino do dispositivo | Pino do Arduino |
|---|---|---|
| TCS3200 | OUT / S2 / S3 | 5 / 6 / 7 |
| Botão | — | 2 e GND |
| Servo da base | sinal | 8 |
| Servo do cotovelo | sinal | 9 |
| Servo do ombro | sinal | 10 |
| Servo da garra | sinal | 13 |
| DFPlayer Mini | TX | 3 |
| DFPlayer Mini | RX (via 1 kΩ) | 4 |
| DFPlayer Mini | VCC / GND | 5 V / GND comum |
| Alto-falante | — | SPK_1 e SPK_2 do DFPlayer |

## Cartão SD

Formatado em FAT32, com uma pasta chamada `mp3` contendo:

| Arquivo | Mensagem |
|---|---|
| `0001.mp3` | Vermelho |
| `0002.mp3` | Azul |
| `0003.mp3` | Verde |
| `0004.mp3` | Cor não identificada |

## Como usar

1. Instale as bibliotecas pelo Gerenciador de Bibliotecas da Arduino IDE:
   - `Servo` (já vem com a IDE)
   - `SoftwareSerial` (já vem com a IDE)
   - `DFRobotDFPlayerMini`
2. Abra `braco_separador_cores/braco_separador_cores.ino` e carregue no Arduino UNO.
3. Abra o Monitor Serial em **9600 baud**.
4. Coloque a peça na frente do sensor, aguarde a cor aparecer no monitor e aperte o botão.

## Ajustes e calibração

Todos os parâmetros ficam no bloco **CONFIGURAÇÃO** no início do código:

- **Poses:** ângulos da base para cada nicho (`BASE_AZUL`, `BASE_VERDE`, `BASE_VERMELHO`) e posições alta/baixa do braço.
- **Velocidade:** `ESPERA_PASSO_MS` (maior = mais lento) e `PAUSA_ETAPA_MS`.
- **Sensor:** `LIMITE_VERMELHO` deve ser ajustado de acordo com a iluminação do ambiente e as peças utilizadas.

## Estrutura do código

| Seção | Responsabilidade |
|---|---|
| Configuração | Pinos, poses, velocidade e parâmetros do sensor |
| Leitura e classificação | Média de pulsos por canal e escolha da cor dominante |
| Áudio | Reprodução das faixas e exibição de erros do DFPlayer |
| Autorização do operador | Botão (com debounce) ou comando serial |
| Movimento suave | Interpolação em "S" com resolução de fração de grau |
| Sequência de separação | Pegar, levar ao nicho, soltar e voltar |

## Trabalhos futuros

O código já traz, comentada, uma implementação de **cinemática inversa** para três juntas (`irParaPonto(x, y, z, garra)`). Depois de medir os elos do braço e calibrar as juntas, ela pode substituir as poses fixas por posições definidas em coordenadas cartesianas.

## Autoria

- Beatriz Cunha
- Melw Vila Nova

**Orientador:** Prof. Dr. Fabio Da Silva Menegatti

Pontifícia Universidade Católica de Campinas — Engenharia de Controle e Automação, 2026.
