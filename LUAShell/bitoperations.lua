---------------------------------------------------
-- Check whether a number is integer
---------------------------------------------------
function check_int(n)
   if (n - math.floor(n) > 0) then
       error("Trying to use bitwise operation on non-integer!")
   end
end

---------------------------------------------------
-- Bitwise OR of two numbers
-- Input is assumed to be 32 bits and output is also 32 bits
---------------------------------------------------
function bit_or(a, b)
    local n1 = to_bits(a) 
    local n2 = to_bits(b)
    expand(n1, n2)

    local n3 = {}
    local res = math.max(table.getn(n1), table.getn(n2))
    
    for i = 1, res do
        if ((n1[i] == 1) or (n2[i] == 1)) then
            n3[i] = 1
        else
            n3[i] = 0
        end
    end
    
    return (tbl_to_number(n3))
end

---------------------------------------------------
-- Bitwise AND of two numbers
-- Input is assumed to be 32 bits and output is also 32 bits
---------------------------------------------------
function bit_and(a, b)
    local n1 = to_bits(a) 
    local n2 = to_bits(b)
    expand(n1, n2)

    local n3 = {}
    local res = math.max(table.getn(n1), table.getn(n2)) 
    
    for i = 1, res do
        if ((n1[i] == 1) and (n2[i] == 1)) then
            n3[i] = 1
        else
            n3[i] = 0
        end
    end
    
    return (tbl_to_number(n3))        
end

---------------------------------------------------
-- Bitwise XOR of two numbers
-- Input is assumed to be 32 bits and output is also 32 bits
---------------------------------------------------
function bit_xor(a, b)
    local n1 = to_bits(a)
    local n2 = to_bits(b)
    expand(n1, n2)

    local n3 = {}
    local res = math.max(table.getn(n1), table.getn(n2)) 
    
    for i = 1, res do
        if (n1[i] ~= n2[i]) then
            n3[i] = 1
        else
            n3[i] = 0
        end
    end
    
    return (tbl_to_number(n3))    
end

---------------------------------------------------
-- Ones complement of a number
---------------------------------------------------
function bit_not(n)
    local tbl = to_bits(n)
    local size = math.max(table.getn(tbl), 32)
    for i = 1, size do
        if (tbl[i] == 1) then 
            tbl[i] = 0
        else
            tbl[i] = 1
        end
    end
 
    return tbl_to_number(tbl)
end

---------------------------------------------------
-- a^b
---------------------------------------------------
function pow(a, b)
    local res = 1
    if (a > 0) then
        for i = 1, b, 1 do
            res = res * a
        end
    end
    return (res)
end

---------------------------------------------------
-- Extract the bits of a integer
---------------------------------------------------
function to_bits(n)
    check_int(n)
    if (n < 0) then
        -- negative
        return to_bits(bit.bnot(math.abs(n)) + 1)
    end

    local tbl = {}
    local cnt = 1
    while (n > 0) do
        local last = math.mod(n, 2)
        if (last == 1) then
            tbl[cnt] = 1
        else
            tbl[cnt] = 0
        end
        n = (n - last)/2
        cnt = cnt + 1
    end

    return(tbl)
end

---------------------------------------------------
-- From a integer from its bits
---------------------------------------------------
function tbl_to_number(tbl)
    local n = table.getn(tbl)
    local rslt = 0
    local power = 1
    for i = 1, n do
        rslt = rslt + tbl[i]*power
        power = power*2
    end
 
    return (rslt)
end

---------------------------------------------------
-- Expand one integer to the size of another integer
---------------------------------------------------
function expand(tbl_m, tbl_n)
    local big = {}
    local small = {}
    
    if (table.getn(tbl_m) > table.getn(tbl_n)) then
        big = tbl_m
        small = tbl_n
    else
        big = tbl_n
        small = tbl_m
    end
 
    -- expand small
    for i = table.getn(small) + 1, table.getn(big) do
        small[i] = 0
    end
end

---------------------------------------------------
-- Left shift a number by n bits
---------------------------------------------------
function bit_lshift(a, n)
    check_int(n)
 
    if (n < 0) then
        -- negative
        n = bit_not(math.abs(n)) + 1
    end

    for i = 1, bits do
      n = n*2
    end
    
    return bit_and(n, 4294967295) -- 0xFFFFFFFF
end

---------------------------------------------------
-- Right shift a number by n bits
---------------------------------------------------
function bit_rshift(n, bits)
    check_int(n)
    local high_bit = 0
 
    if (n < 0) then
        -- negative
        n = bit_not(math.abs(n)) + 1
        high_bit = 2147483648 -- 0x80000000
    end

    for i = 1, bits do
        n = n/2
        n = bit_or(math.floor(n), high_bit)
    end

    return (math.floor(n))
end

---------------------------------------------------
-- Logical rightshift assures zero filling shift
---------------------------------------------------
local function bit_logic_rshift(n, bits)
    check_int(n)
    
    if (n < 0) then
        -- negative
        n = bit_not(math.abs(n)) + 1
    end
 
    for i = 1, bits do
        n = n/2
    end
    
    return math.floor(n)
end

---------------------------------------------------
-- Read a few bits from a variable
---------------------------------------------------
function bit_extract(val, en_bit, st_bit)
   local nbits       = en_bit - st_bit + 1
   local mask        = math.pow(2, nbits) - 1
   local res

   res = bit_and(bit_rshift(val, st_bit), mask)

   return (res)   
end

---------------------------------------------------
-- Read a few bits from a register
---------------------------------------------------
function reg_read(addr, en_bit, st_bit)
   local ret, regVal = ar1.Calling_ReadAddr_Single(addr)
   local nbits       = en_bit - st_bit + 1
   local mask        = math.pow(2, nbits) - 1

   RSTD.Log(string.format("RegVal = 0x%08X, MASK = 0x%08X\n", regVal, mask), "green")

   regVal = bit_and(bit_rshift(regVal, st_bit), mask)

   return (regVal)
end

---------------------------------------------------
-- Write a few bits to a register
---------------------------------------------------
function reg_write(addr, en_bit, st_bit, val)
   local regVal = ar1.Calling_ReadAddr_Single(addr)
   local nbits  = en_bit - st_bit + 1
   local mask   = math.pow(2, nbits) - 1

   regVal = bit_and(regVal, bit_lshift(mask, st_bit)) 
   wrVal  = bit_lshift(bit_and(val, mask), st_bit)

   regVal = bit_or(regVal, wrVal)

   ar1.Calling_WriteAddr_Single(addr, regVal)
end





