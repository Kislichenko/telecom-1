packet=[0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 0 0 0 0 0 0 0 0 0 1 0 1 1 1 0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]

PRS=[1;  1;  1;  1;  1;  1; -1; -1; -1; -1;  1;  1;  1;  1; -1;  1;  1;  1; -1; -1; -1; -1;  1; -1;  1;  1; -1; -1;  1;  1; -1;  1;
          1; -1;  1;  1;  1;  1; -1;  1; -1; -1; -1; -1;  1;  1;  1; -1; -1;  1;  1; -1; -1; -1; -1;  1; -1; -1;  1; -1; -1; -1;  1; -1;
          1; -1;  1;  1;  1; -1;  1; -1;  1;  1;  1;  1; -1; -1;  1; -1; -1;  1; -1;  1;  1;  1; -1; -1;  1;  1;  1; -1; -1; -1; -1; -1;
         -1;  1;  1;  1; -1;  1;  1;  1; -1;  1; -1; -1;  1;  1;  1;  1; -1;  1; -1;  1; -1; -1;  1; -1;  1; -1; -1; -1; -1; -1; -1;  1;
         -1;  1; -1;  1; -1;  1; -1;  1;  1;  1;  1;  1; -1;  1; -1;  1;  1; -1;  1; -1; -1; -1; -1; -1;  1;  1; -1;  1;  1;  1; -1;  1;
          1; -1;  1;  1; -1;  1; -1;  1;  1; -1; -1; -1; -1; -1;  1; -1;  1;  1;  1; -1;  1;  1;  1;  1;  1; -1; -1; -1;  1;  1;  1;  1;
         -1; -1;  1;  1; -1;  1; -1; -1;  1;  1; -1;  1; -1;  1;  1;  1; -1; -1; -1;  1;  1; -1;  1; -1; -1; -1;  1; -1;  1;  1;  1;  1;
          1;  1;  1; -1;  1; -1; -1;  1; -1;  1;  1; -1; -1; -1;  1; -1;  1; -1; -1;  1;  1; -1; -1; -1;  1;  1; -1; -1; -1; -1; -1; -1;
         -1;  1;  1; -1; -1;  1;  1; -1; -1;  1; -1;  1; -1;  1;  1; -1; -1;  1; -1; -1;  1;  1;  1;  1;  1;  1; -1;  1;  1; -1;  1; -1;
         -1;  1; -1; -1;  1; -1; -1;  1;  1; -1;  1;  1;  1;  1;  1;  1; -1; -1;  1; -1;  1;  1; -1;  1; -1;  1; -1; -1; -1; -1;  1; -1;
          1; -1; -1; -1;  1; -1; -1;  1;  1;  1; -1;  1;  1; -1; -1;  1; -1;  1;  1;  1;  1; -1;  1;  1; -1; -1; -1; -1;  1;  1; -1;  1;
         -1;  1; -1;  1; -1; -1;  1;  1;  1; -1; -1;  1; -1; -1; -1; -1;  1;  1; -1; -1; -1;  1; -1; -1; -1; -1;  1; -1; -1; -1; -1; -1;
         -1; -1; -1;  1; -1; -1; -1;  1; -1; -1; -1;  1;  1; -1; -1;  1; -1; -1; -1;  1;  1;  1; -1;  1; -1;  1; -1;  1;  1; -1;  1;  1;
         -1; -1; -1;  1;  1;  1; -1; -1; -1;  1; -1; -1;  1; -1;  1; -1;  1; -1; -1; -1;  1;  1; -1;  1;  1; -1; -1;  1;  1;  1;  1;  1;
         -1; -1;  1;  1;  1;  1; -1; -1; -1;  1; -1;  1;  1; -1;  1;  1;  1; -1; -1;  1; -1;  1; -1; -1;  1; -1; -1; -1; -1; -1;  1; -1;
         -1;  1;  1; -1; -1;  1;  1;  1; -1;  1; -1; -1; -1;  1;  1;  1;  1;  1; -1;  1;  1;  1;  1; -1; -1; -1; -1; -1;  1;  1;  1  ];

 
%convolution encoder
tr1=poly2trellis(9,[753 561]);
convcode=convenc(packet,tr1)
%interleaver
 interleaver = [0; 133; 122; 111; 100; 89; 78; 67; 56; 45; 34; 23; 12; 1; 134; 123; 
	112; 101; 90; 79; 68; 57; 46; 35; 24; 13; 2; 135; 124; 113; 102; 91; 
	80; 69; 58; 47; 36; 25; 14; 3; 136; 125; 114; 103; 92; 81; 70; 59; 
	48; 37; 26; 15; 4; 137; 126; 115; 104; 93; 82; 71; 60; 49; 38; 27; 
	16; 5; 138; 127; 116; 105; 94; 83; 72; 61; 50; 39; 28; 17; 6; 139; 
	128; 117; 106; 95; 84; 73; 62; 51; 40; 29; 18; 7; 140; 129; 118; 107; 
	96; 85; 74; 63; 52; 41; 30; 19; 8; 141; 130; 119; 108; 97; 86; 75; 
	64; 53; 42; 31; 20; 9; 142; 131; 120; 109; 98; 87; 76; 65; 54; 43; 
	32; 21; 10; 143; 132; 121; 110; 99; 88; 77; 66; 55; 44; 33; 22; 11];

sig = convcode(int16(interleaver+1));
sig_matrix=reshape(sig(1:144),[6 24])';  %было неправильн. перемежение

% create Walsh matrix 64x64
x=1;
P=6;%log walsh matrix dimension
for i=1:1:P
    temp=x;
    x=[temp temp;temp -temp];
    i=i+1;
end;
%take Walsh matrix row by 6-bit number
y=hadamard(64);
Walsh_row_number(24);
line=sig_matrix(24,:);
for k=1:1:24
    line=sig_matrix(k,:);
    Walsh_row_number(k)=line(1)*(2^5)+line(2)*(2^4)+line(3)*(2^3)+line(4)*(2^2)+line(5)*(2^1)+line(6)*(2^0)+1;
    signal3(k,:)=fwht(sig_matrix(k,:),64);
   end;
   size(sig_matrix);
Walsh_row_number;
for i1=1:1:24
    for j1=1:1:64
        if signal3(i1,j1)>=0
            signal3(i1,j1)=1;
        else signal3(i1,j1)=-1;
        end;
    end;
end;


 %*******************************************************
 % Walsh matrix generation by Hadamard matrix index rearrangement
 % http://www.mathworks.com/help/signal/examples/discrete-walsh-hadamard-transform.html
 N=64;
 hadamardMatrix=hadamard(N);

 HadIdx = 0:N-1;                          % Hadamard index
 M = log2(N)+1;  
 
 binHadIdx = fliplr(dec2bin(HadIdx,M))-'0'; % Bit reversing of the binary index
 binSeqIdx = zeros(N,M-1);                  % Pre-allocate memory
for k = M:-1:2
    % Binary sequency index
    binSeqIdx(:,k) = xor(binHadIdx(:,k),binHadIdx(:,k-1));
end
SeqIdx = binSeqIdx*pow2((M-1:-1:0)');    % Binary to integer sequency index
walshMatrix = hadamardMatrix(SeqIdx+1,:); % 1-based indexing
 %*******************************************************
 
 signal=x(Walsh_row_number,:);
 signal1=hadamardMatrix(Walsh_row_number,:);
 signal2=walshMatrix(Walsh_row_number,:);
 
    signal_to_modulate1=reshape(signal2',[1 1536]);
    signal_to_modulate2= signal_to_modulate1.*[PRS' PRS' PRS' PRS(1:3)'];
    signal_to_modulate=[PRS' signal_to_modulate2];
    
%convert demodulation result to unipolar form and bpsk modulate

for u=1:1:length(signal_to_modulate)
     if (signal_to_modulate(u)==-1)
        signal_to_modulate(u)=0;
     else signal_to_modulate(u)=1;
     end;
end;

IQ=pskmod(signal_to_modulate,2);
%add white gaussian noise
 IQ_1=awgn(IQ,-6);
IQ_1=IQ;
scatterplot(IQ_1);

% model oversampling by 2
a=[IQ_1; IQ_1];
b=reshape(a,[1 4094]);
im_part=imag(b);
re_part=real(b);
c=[re_part; im_part];
% figure(2);
IQ_record1=reshape(c, [1 8188]);
IQ_record=int16(IQ_record1);

IQ_record= [IQ_record(101:180) IQ_record];

%calculate CRC-code
hDetect=crc.generator('Polynomial', '0x1021');
 decode_with_crc=generate(hDetect,packet');
 crc_send=decode_with_crc(73:88);
 
% write to file
 fid=fopen('C:\test1.sig', 'w','l');
 fwrite(fid,  IQ_record, 'int16');
 fclose(fid);
 
 
 
 
 
 
 
 
 
 
 

